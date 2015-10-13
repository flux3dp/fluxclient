
import readline
import curses

class Console(object):
    def __enter__(self):
        self.stdscr = curses.initscr()
        return self

    def __exit__(self, type, value, tb):
        curses.endwin()

    def setup(self):
        curses.start_color()
        curses.init_pair(1, curses.COLOR_WHITE, curses.COLOR_BLUE)

        curses.cbreak()
        curses.echo()

        lines, cols = self.stdscr.getmaxyx()
        self.logwin = self.stdscr.subwin(lines - 2, cols, 0, 0)
        self.sepwin = self.stdscr.subwin(1, cols, lines - 2, 0)
        self.cmdwin = self.stdscr.subwin(1, cols, lines - 1, 0)

        self.logwin.scrollok(True)

        self.sepwin.bkgd(curses.color_pair(1))
        self.sepwin.refresh()

    def append_log(self, buf):
        self.logwin.addstr(buf)
        self.logwin.refresh()

    def write(self, buf):
        self.append_log(buf)

    def flush(self):
        pass

    def read_cmd(self):
        lines, cols = self.stdscr.getmaxyx()

        self.cmdwin.addstr(b" " + b"\r")
        self.cmdwin.refresh()
        cmd = input()
        self.logwin.redrawwin()
        self.cmdwin.redrawwin()
        self.sepwin.redrawwin()
        self.append_log("> %s\n" % cmd)
        return cmd

if __name__ == "__main__":
    with Console() as i:
        i.setup()
        while True:
            lines, cols = i.stdscr.getmaxyx()
            buf = i.read_cmd()
            i.write(buf + "\n")
