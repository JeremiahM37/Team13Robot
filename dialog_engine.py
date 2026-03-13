"""
Dialog Engine - TangoChat-style script parser and conversation engine.

Parses DSL script files and runs rule-based conversations with:
- Definitions (~name), bracket choices, variable capture
- Scoped subrules (u:, u1:, u2:, ...)
- Action tags (<head_yes>, <head_no>, <arm_raise>, <dance90>)
- Safety interrupts and state machine

Usage:
    engine = DialogEngine()
    engine.load_script("script.txt")
    speak_text, actions = engine.process_input("hello")
"""

import re
import random


# Max nesting depth guard
MAX_NESTING_DEPTH = 6


class ParseError:
    """Represents a parsing error with location info."""
    def __init__(self, filename, line_num, category, message, fatal=False):
        self.filename = filename
        self.line_num = line_num
        self.category = category
        self.message = message
        self.fatal = fatal

    def __str__(self):
        severity = "FATAL" if self.fatal else "WARNING"
        return f"[{severity}] {self.filename}:{self.line_num} ({self.category}) {self.message}"


class Rule:
    """A single conversation rule with pattern, output, and optional subrules."""
    def __init__(self, level, pattern_str, output_str, line_num):
        self.level = level          # 0 for u:, 1 for u1:, 2 for u2:, etc.
        self.pattern_str = pattern_str
        self.output_str = output_str
        self.line_num = line_num
        self.subrules = []          # List of Rule objects nested under this one

    def __repr__(self):
        return f"Rule(L{self.level}, pattern={self.pattern_str!r}, subs={len(self.subrules)})"


class DialogEngine:
    """
    Parses TangoChat-style scripts and runs conversations.
    Produces (speak_text, action_list) tuples — does NOT execute actions itself.
    """

    # Safety interrupt words
    INTERRUPT_WORDS = {'stop', 'cancel', 'reset', 'quit'}

    def __init__(self, seed=None):
        self.definitions = {}       # ~name -> list of options
        self.top_rules = []         # Top-level u: rules
        self.variables = {}         # Captured variables ($name)
        self.errors = []            # Parse errors

        # State machine
        self.state = 'BOOT'
        self.active_scope = []      # Stack of matched rules for scope tracking
        self.unmatched_count = 0    # Consecutive unmatched inputs in scope

        # Deterministic mode
        self.rng = random.Random(seed)

        self._filename = '<unknown>'

    # ==================== PARSING ====================

    def load_script(self, filename):
        """Load and parse a script file. Returns True if engine is runnable."""
        self._filename = filename
        self.errors = []
        self.definitions = {}
        self.top_rules = []

        try:
            with open(filename, 'r') as f:
                lines = f.readlines()
        except FileNotFoundError:
            err = ParseError(filename, 0, 'file', f'File not found: {filename}', fatal=True)
            self.errors.append(err)
            print(err)
            return False
        except Exception as e:
            err = ParseError(filename, 0, 'file', str(e), fatal=True)
            self.errors.append(err)
            print(err)
            return False

        self._parse_lines(lines)

        # Print all errors
        for err in self.errors:
            print(err)

        # Fatal check: must have at least one valid top-level u: rule
        if not self.top_rules:
            err = ParseError(filename, 0, 'structure', 'No valid top-level u: rules found', fatal=True)
            self.errors.append(err)
            print(err)
            return False

        # Check for any fatal errors
        if any(e.fatal for e in self.errors):
            return False

        self.state = 'IDLE'
        print(f"[STATE] BOOT -> IDLE")
        print(f"Loaded {filename}: {len(self.top_rules)} top-level rules, "
              f"{len(self.definitions)} definitions")
        return True

    def _parse_lines(self, lines):
        """Parse all lines, building definitions and rule tree."""
        # First pass: collect all lines with their numbers, strip comments/blanks
        parsed_lines = []
        for i, line in enumerate(lines, 1):
            # Strip comments (everything from # to end of line)
            comment_pos = line.find('#')
            if comment_pos >= 0:
                line = line[:comment_pos]

            stripped = line.rstrip()
            if not stripped.strip():
                continue

            parsed_lines.append((i, stripped))

        # Second pass: parse definitions and rules
        rule_stack = []  # Stack of (indent_level, rule) for building tree

        for line_num, line in parsed_lines:
            stripped = line.strip()

            # Definition: ~name: [options]
            def_match = re.match(r'~(\w+)\s*:\s*\[(.+)\]', stripped)
            if def_match:
                name = def_match.group(1)
                options_str = def_match.group(2)
                options = self._parse_bracket_options(options_str)
                if options:
                    self.definitions[name] = options
                else:
                    self.errors.append(ParseError(
                        self._filename, line_num, 'definition',
                        f'Empty definition for ~{name}'))
                continue

            # Rule: u:(pattern): output  OR  u1:(pattern): output
            # The format has TWO colons: one after u/u1, one after the closing paren
            rule_match = re.match(r'(\s*)u(\d*)\s*:\s*\((.+?)\)\s*:\s*(.*)', line)
            if not rule_match:
                # Also try on stripped version
                rule_match = re.match(r'(\s*)u(\d*)\s*:\s*\((.+?)\)\s*:\s*(.*)', stripped)

            if rule_match:
                indent = len(line) - len(line.lstrip())
                level_str = rule_match.group(2)
                pattern_str = rule_match.group(3).strip()
                output_str = rule_match.group(4).strip()

                level = int(level_str) if level_str else 0

                if not output_str:
                    self.errors.append(ParseError(
                        self._filename, line_num, 'rule',
                        f'Rule has no output', fatal=False))
                    continue

                # Check for unbalanced brackets in output
                if output_str.count('[') != output_str.count(']'):
                    self.errors.append(ParseError(
                        self._filename, line_num, 'syntax',
                        f'Unbalanced brackets in output: {output_str!r}', fatal=False))
                    continue

                rule = Rule(level, pattern_str, output_str, line_num)

                if level == 0:
                    # Top-level rule
                    self.top_rules.append(rule)
                    rule_stack = [(indent, rule)]
                else:
                    # Subrule — attach to parent
                    parent_found = False
                    for j in range(len(rule_stack) - 1, -1, -1):
                        _, parent_rule = rule_stack[j]
                        if parent_rule.level == level - 1:
                            parent_rule.subrules.append(rule)
                            rule_stack = rule_stack[:j + 1]
                            rule_stack.append((indent, rule))
                            parent_found = True
                            break

                    if not parent_found:
                        self.errors.append(ParseError(
                            self._filename, line_num, 'scope',
                            f'u{level}: has no parent u{level-1}: rule', fatal=False))
                continue

            # Check if it looks like a rule but with wrong format (missing second colon)
            bad_rule = re.match(r'\s*u\d*\s*:\s*\(.+?\)\s+\S', stripped)
            if bad_rule:
                self.errors.append(ParseError(
                    self._filename, line_num, 'syntax',
                    f'Rule missing colon after pattern: {stripped!r}', fatal=False))
                continue

            # Check if it looks like a bad definition (missing colon)
            bad_def = re.match(r'~\w+\s+\[', stripped)
            if bad_def:
                self.errors.append(ParseError(
                    self._filename, line_num, 'syntax',
                    f'Definition missing colon: {stripped!r}', fatal=False))
                continue

            # Unrecognized line
            self.errors.append(ParseError(
                self._filename, line_num, 'syntax',
                f'Unrecognized line: {stripped!r}', fatal=False))

    def _parse_bracket_options(self, text):
        """Parse bracket options like: a b c "two words" into a list."""
        options = []
        i = 0
        current = ''
        while i < len(text):
            ch = text[i]
            if ch == '"':
                # Quoted string
                end = text.find('"', i + 1)
                if end == -1:
                    end = len(text)
                options.append(text[i + 1:end])
                i = end + 1
            elif ch in (' ', '\t'):
                if current:
                    options.append(current)
                    current = ''
                i += 1
            else:
                current += ch
                i += 1
        if current:
            options.append(current)
        return options

    # ==================== MATCHING ====================

    def _clean_input(self, text):
        """Normalize user input: lowercase, strip punctuation."""
        text = text.lower().strip()
        text = re.sub(r'[.,!?;:\'"]+', '', text)
        text = re.sub(r'\s+', ' ', text)
        return text

    def _clean_input_preserve_case(self, text):
        """Strip punctuation and normalize whitespace but preserve case."""
        text = text.strip()
        text = re.sub(r'[.,!?;:\'"]+', '', text)
        text = re.sub(r'\s+', ' ', text)
        return text

    def _expand_definition(self, token):
        """If token is ~name, return the list of options. Otherwise return [token]."""
        if token.startswith('~') and token[1:] in self.definitions:
            return self.definitions[token[1:]]
        return [token]

    def _build_pattern_regex(self, pattern_str):
        """
        Build a regex from a pattern string.
        Supports:
          - [a b c "two words"] bracket choices
          - ~name definitions (expanded into choices)
          - _ wildcard (captures any text)
          - * wildcard (matches any text, no capture)
          - literal words
        Returns (compiled_regex, list_of_capture_group_indices).
        """
        tokens = self._tokenize_pattern(pattern_str)
        regex_parts = []
        capture_indices = []
        group_idx = 0

        for token in tokens:
            if token == '_':
                # Wildcard capture
                group_idx += 1
                capture_indices.append(group_idx)
                regex_parts.append(r'(.+?)')
            elif token == '*':
                # Non-capturing wildcard
                regex_parts.append(r'(?:.+?)')
            elif token.startswith('[') and token.endswith(']'):
                # Bracket choices
                inner = token[1:-1]
                options = self._parse_bracket_options(inner)
                # Expand definitions within options
                expanded = []
                for opt in options:
                    expanded.extend(self._expand_definition(opt))
                escaped = [re.escape(o) for o in expanded]
                group_idx += 1
                regex_parts.append(f'({"|".join(escaped)})')
            elif token.startswith('~'):
                # Definition reference
                options = self._expand_definition(token)
                escaped = [re.escape(o) for o in options]
                group_idx += 1
                regex_parts.append(f'({"|".join(escaped)})')
            else:
                # Literal token — may contain spaces (from quoted strings)
                words = token.split()
                if len(words) > 1:
                    regex_parts.append(r'\s+'.join(re.escape(w) for w in words))
                else:
                    regex_parts.append(re.escape(token))

        pattern = r'^\s*' + r'\s+'.join(regex_parts) + r'\s*$'
        return re.compile(pattern, re.IGNORECASE), capture_indices

    def _tokenize_pattern(self, pattern_str):
        """Split a pattern string into tokens, respecting brackets and quotes."""
        tokens = []
        i = 0
        current = ''

        while i < len(pattern_str):
            ch = pattern_str[i]
            if ch == '"':
                # Quoted phrase — treat as single literal token
                if current:
                    tokens.append(current)
                    current = ''
                end = pattern_str.find('"', i + 1)
                if end == -1:
                    end = len(pattern_str)
                # Store without quotes — will be matched as a literal phrase
                tokens.append(pattern_str[i + 1:end])
                i = end + 1
            elif ch == '[':
                if current:
                    tokens.append(current)
                    current = ''
                # Find matching ]
                depth = 1
                j = i + 1
                while j < len(pattern_str) and depth > 0:
                    if pattern_str[j] == '[':
                        depth += 1
                    elif pattern_str[j] == ']':
                        depth -= 1
                    j += 1
                tokens.append(pattern_str[i:j])
                i = j
            elif ch in (' ', '\t'):
                if current:
                    tokens.append(current)
                    current = ''
                i += 1
            else:
                current += ch
                i += 1

        if current:
            tokens.append(current)
        return tokens

    def _match_rule(self, rule, user_input):
        """
        Try to match user_input against a rule's pattern.
        Returns captured wildcard text list if matched, None otherwise.
        """
        regex, capture_indices = self._build_pattern_regex(rule.pattern_str)
        match = regex.match(user_input)
        if match:
            captures = [match.group(idx) for idx in capture_indices]
            return captures
        return None

    # ==================== OUTPUT PROCESSING ====================

    def _resolve_output(self, output_str, captures):
        """
        Process an output string:
        - Extract action tags <...>
        - Resolve bracket choices [a b c] -> random pick
        - Resolve $variable references
        - Store captures into variables if pattern had _ with $name in output
        Returns (speak_text, action_list).
        """
        # Extract action tags
        actions = re.findall(r'<(\w+)>', output_str)
        text = re.sub(r'<\w+>', '', output_str)

        # Resolve bracket choices in output — find [...] respecting quotes
        text = self._resolve_bracket_choices(text)

        # Handle variable capture: if captures exist, look for $var patterns
        # The convention is: if pattern has _ and output has $name,
        # the first _ maps to the first $name, etc.
        var_refs = re.findall(r'\$(\w+)', text)
        for i, var_name in enumerate(var_refs):
            if i < len(captures):
                self.variables[var_name] = captures[i]

        # Resolve $variable references
        def replace_var(m):
            var_name = m.group(1)
            if var_name in self.variables:
                return self.variables[var_name]
            return "I don't know"

        text = re.sub(r'\$(\w+)', replace_var, text)

        # Expand ~definitions in output text
        def replace_def(m):
            name = m.group(1)
            if name in self.definitions:
                return self.rng.choice(self.definitions[name])
            return m.group(0)

        text = re.sub(r'~(\w+)', replace_def, text)

        # Clean up whitespace
        text = re.sub(r'\s+', ' ', text).strip()

        return text, actions

    def _resolve_bracket_choices(self, text):
        """Resolve [a b c "two words"] bracket choices in output text."""
        result = ''
        i = 0
        while i < len(text):
            if text[i] == '[':
                # Find matching ]
                depth = 1
                j = i + 1
                while j < len(text) and depth > 0:
                    if text[j] == '[':
                        depth += 1
                    elif text[j] == ']':
                        depth -= 1
                    j += 1
                if depth == 0:
                    inner = text[i + 1:j - 1]
                    options = self._parse_bracket_options(inner)
                    # Expand definitions in options
                    expanded = []
                    for opt in options:
                        if opt.startswith('~') and opt[1:] in self.definitions:
                            expanded.extend(self.definitions[opt[1:]])
                        else:
                            expanded.append(opt)
                    result += self.rng.choice(expanded)
                    i = j
                else:
                    # Unbalanced bracket, keep literal
                    result += text[i]
                    i += 1
            else:
                result += text[i]
                i += 1
        return result

    # ==================== STATE MACHINE ====================

    def process_input(self, user_input):
        """
        Process user input and return (speak_text, action_list).
        Manages the state machine and scope.
        """
        if self.state == 'BOOT':
            return "Engine not loaded. Please load a script first.", []

        cleaned = self._clean_input(user_input)
        preserved = self._clean_input_preserve_case(user_input)

        # Check for safety interrupt
        if cleaned in self.INTERRUPT_WORDS:
            old_state = self._state_str()

            # Try to find a matching rule for the response text
            response_text = "OK. Stopping now."
            for rule in self.top_rules:
                result = self._match_rule(rule, cleaned)
                if result is not None:
                    response_text, _ = self._resolve_output(rule.output_str, result)
                    print(f"[MATCH] Rule at line {rule.line_num}: "
                          f"({rule.pattern_str}) -> {rule.output_str}")
                    break

            self.active_scope = []
            self.unmatched_count = 0
            self.state = 'IDLE'
            print(f"[SAFETY] Interrupt '{cleaned}' received")
            print(f"[STATE] {old_state} -> IDLE")
            return response_text, ['__stop__']

        # Determine which rules to try
        active_rules = self._get_active_rules()

        # Try matching
        matched_rule = None
        captures = []
        for rule in active_rules:
            result = self._match_rule(rule, cleaned)
            if result is not None:
                matched_rule = rule
                # Re-match against case-preserved input for captures
                preserved_result = self._match_rule(rule, preserved)
                captures = preserved_result if preserved_result is not None else result
                break

        if matched_rule:
            self.unmatched_count = 0
            print(f"[MATCH] Rule at line {matched_rule.line_num}: "
                  f"({matched_rule.pattern_str}) -> {matched_rule.output_str}")

            # Update scope
            old_state = self._state_str()

            if matched_rule.level == 0:
                # Top-level match clears previous scope
                self.active_scope = [matched_rule]
            else:
                # Subrule match: set scope to this rule
                # Trim scope to parent level, then add this rule
                self.active_scope = self.active_scope[:matched_rule.level]
                self.active_scope.append(matched_rule)

            # Check max nesting depth
            if len(self.active_scope) > MAX_NESTING_DEPTH:
                print(f"[SAFETY] Max nesting depth ({MAX_NESTING_DEPTH}) exceeded, resetting")
                self.active_scope = []
                self.state = 'IDLE'
                print(f"[STATE] {old_state} -> IDLE")
                return "I need to reset, that got too deep.", ['__stop__']

            # Update state
            if matched_rule.subrules:
                depth = len(self.active_scope)
                self.state = f'IN_SCOPE({depth})'
            elif len(self.active_scope) > 1:
                depth = len(self.active_scope) - 1
                self.state = f'IN_SCOPE({depth})'
            else:
                self.state = 'IDLE'

            print(f"[STATE] {old_state} -> {self._state_str()}")

            # Resolve output
            speak_text, actions = self._resolve_output(matched_rule.output_str, captures)

            if actions:
                print(f"[STATE] {self._state_str()} -> EXEC_ACTIONS")
                for a in actions:
                    print(f"[ACTION] Queued: {a}")

            return speak_text, actions

        else:
            # No match
            if self.active_scope:
                self.unmatched_count += 1
                print(f"[NO MATCH] Unmatched in scope ({self.unmatched_count}/4)")

                if self.unmatched_count >= 4:
                    old_state = self._state_str()
                    self.active_scope = []
                    self.unmatched_count = 0
                    self.state = 'IDLE'
                    print(f"[SAFETY] 4 consecutive unmatched inputs, resetting scope")
                    print(f"[STATE] {old_state} -> IDLE")
                    return "I'm not sure what you mean. Let's start over.", []
            else:
                print(f"[NO MATCH] No matching rule found")

            return "I don't understand that.", []

    def _get_active_rules(self):
        """Get the list of rules to try matching against, based on current scope."""
        if not self.active_scope:
            return self.top_rules

        # Get subrules of the deepest scoped rule, plus top-level rules as fallback
        deepest = self.active_scope[-1]
        if deepest.subrules:
            return deepest.subrules + self.top_rules
        else:
            # No subrules at this level, try parent's subrules or top-level
            for i in range(len(self.active_scope) - 2, -1, -1):
                parent = self.active_scope[i]
                if parent.subrules:
                    return parent.subrules + self.top_rules
            return self.top_rules

    def _state_str(self):
        """Get current state as string."""
        return self.state

    def get_state(self):
        """Get current state for external use."""
        return self.state

    def get_scope_depth(self):
        """Get current scope depth."""
        return len(self.active_scope)
