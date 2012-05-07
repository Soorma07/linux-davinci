import os
import re
import sys

config_tags = [ ]

class ConfigTag:

    (BOOL, TRISTATE, INT, HEX, STRING, RANGE) = range(6)

    def __init__(self, name):
        self.name = name
        self.help_lines = [ ]
        self.type = None
        self.legend = None
        self.depends = [ ]
        self.selects = [ ]
        self.condition = None
        self.choice = None

    def parse_dependencies(self, x):
        #print x
        pass

class ChoiceTag(ConfigTag):
    pass

class CommentTag(ConfigTag):
    pass

class MenuTag(ConfigTag):
    pass

class MainMenuTag(MenuTag):
    pass

class MenuConfigTag(MenuTag):
    pass


class KconfigParser:

    def __init__(self):
        self.tag = None
        self.help_mode = False
        self.if_mode = None
        self.choice = None
        self.handlers = [ ]
        for name in dir(self):
            if name.startswith('handle_'):
                method = getattr(self, name)
                r = re.compile('^' + method.__doc__)
                self.handlers.append((r.search, method))

    def parse_line(self, x, fname, linenum):
        if len(x.strip()) == 0:
            return

        if self.help_mode:
            if x[:1] not in ' \t':
                self.help_mode = False
            else:
                self.tag.help_lines.append(x.strip())
                return

        x = x.strip()

        for pattern, method in self.handlers:
            if pattern(x) is not None:
                method(x)
                return

        # not help, and not one of the recognized patterns
        raise Exception, (fname, linenum, x)

    def close_tag(self):
        if self.tag is not None:
            config_tags.append(self.tag)
            self.tag = None

    def handle_comment(self, x):
        "comment\s"
        self.close_tag()
        self.tag = CommentTag(x[8:].strip())
        self.help_mode = False
        self.tag.condition = self.if_mode

    def handle_mainmenu(self, x):
        "mainmenu\s"
        self.close_tag()
        self.tag = MainMenuTag(x[9:].strip())
        self.help_mode = False
        self.tag.condition = self.if_mode

    def handle_menuconfig(self, x):
        "menuconfig\s"
        self.close_tag()
        self.tag = MenuConfigTag(x[11:].strip())
        self.if_mode = None  # ? ? ?
        self.help_mode = False

    def handle_config(self, x):
        "config\s"
        self.close_tag()
        self.tag = ConfigTag(x[7:].strip())
        self.help_mode = False
        self.tag.condition = self.if_mode

    #############################

    def handle_if(self, x):
        "if"
        self.if_mode = x[2:].strip()

    def handle_endif(self, x):
        "endif"
        self.if_mode = None

    #############################

    def handle_choice(self, x):
        "choice"
        self.close_tag()
        self.tag = ChoiceTag(x[7:].strip())
        self.help_mode = False
        self.tag.condition = self.if_mode

    def handle_endchoice(self, x):
        "endchoice"
        # TODO
        pass

    #############################

    def handle_menu(self, x):
        "menu\s"
        self.close_tag()
        self.tag = MenuTag(x[5:].strip())
        self.help_mode = False
        self.tag.condition = self.if_mode

    def handle_endmenu(self, x):
        "endmenu"
        # TODO
        pass

    #############################

    def handle_source(self, x):
        "source\s"
        # TODO
        pass

    #############################

    def handle_bool(self, x):
        "bool"
        self.tag.type = ConfigTag.BOOL
        self.tag.legend = x[5:].strip()

    def handle_tristate(self, x):
        "tristate"
        self.tag.type = ConfigTag.TRISTATE
        self.tag.legend = x[9:].strip()

    def handle_int(self, x):
        "int"
        self.tag.type = ConfigTag.INT
        self.tag.legend = x[4:].strip()

    def handle_hex(self, x):
        "hex"
        self.tag.type = ConfigTag.HEX
        self.tag.legend = x[4:].strip()

    def handle_string(self, x):
        "string"
        self.tag.type = ConfigTag.STRING
        # I assume legend becomes the string value?

    def handle_range(self, x):
        "range"
        self.tag.type = ConfigTag.RANGE
        # TODO save min, max, and possible condition

    # a prompt seems to be part of a choice, but it may
    # include a conditional, like
    # "JFFS2 default compression mode" if JFFS2_COMPRESSION_OPTIONS
    def handle_prompt(self, x):
        "prompt"
        self.tag.legend = x[7:].strip()
        # TODO
        pass

    #############################

    def handle_depends(self, x):
        "depends on\s"
        x = x[12:].strip()
        # handle multi-line dependency statements with
        # backslashes on all but the last line
        if x.endswith('\\'):
            self.accumulate = x[-1]
            def line_consumer(x, owner=self):
                if x.endswith('\\'):
                    owner.accumulate += ' ' + x[-1]
                else:
                    x = owner.accumulate + ' ' + x
                    owner.tag.parse_dependencies(x)
                    owner.line_consumer = None
            self.line_consumer = line_consumer
        else:
            self.tag.parse_dependencies(x)

    def handle_visible_if(self, x):
        "visible if\s"
        # TODO
        pass

    def handle_optional(self, x):
        "optional"
        # TODO
        pass

    def handle_select(self, x):
        "select\s"
        self.tag.selects.append(x[8:].strip())

    def handle_option(self, x):
        "option\s"
        #self.tag.selects.append(x[8:].strip())
        # TODO
        pass

    def handle_default(self, x):
        "default\s"
        # TODO
        pass

    def handle_def_bool(self, x):
        "def_bool\s"
        # TODO
        pass

    def handle_def_tristate(self, x):
        "def_tristate\s"
        # TODO
        pass

    #############################

    def handle_help(self, x):
        "(help|---help---|--- help ---)"
        self.help_mode = True


kp = KconfigParser()

for root, dirs, files in os.walk('.'):
    if 'Kconfig' in files:
        fname = os.path.join(root, 'Kconfig')
        inf = open(fname)
        accum = [ ]
        linenum = 1
        for L in inf.readlines():
            try:
                n = L.index('#')
                L = L[:n]
            except ValueError:
                pass
            L = L.rstrip()
            if L[-1:] == '\\':
                accum.append(L[:-1])
            else:
                accum.append(L)
                accum = map(lambda x: x.rstrip(), accum[:1]) + \
                    map(lambda x: x.strip(), accum[1:])
                kp.parse_line(' '.join(accum), fname, linenum)
                accum = [ ]
            linenum += 1


print config_tags
