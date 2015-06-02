
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

    def read_cmd(self):
        lines, cols = self.stdscr.getmaxyx()
        cmd = self.stdscr.getstr(lines - 1, 0).decode("utf8")
        self.cmdwin.addstr(b" " * len(cmd) + b"\r")
        self.cmdwin.refresh()

        self.append_log("> %s\n" % cmd)
        return cmd

if __name__ == "__main__":
    def dadada(iv):
        from time import sleep
        while True:
            iv.logwin.addstr("dadada\n")
            iv.logwin.refresh()
            sleep(1.0)

    with Console() as i:
        i.setup()

        import threading
        t = threading.Thread(target=dadada, args=(i, ))
        t.setDaemon(True)
        t.start()

        while True:
            lines, cols = i.stdscr.getmaxyx()
            buf = i.stdscr.getstr(lines - 1, 0)
            i.append_log(buf)
