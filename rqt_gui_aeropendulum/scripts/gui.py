#!/usr/bin/env python

#!/usr/bin/env python

import sys

from rqt_gui.main import Main

main = Main()
sys.exit(main.main(sys.argv, standalone='rqt_gui_aeropendulum.aeropendulum_gui.Aeropendulum'))