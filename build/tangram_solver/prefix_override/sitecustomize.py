import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jingkun/Documents/Final_Project/src/Autonomous_Tangram_Solver/install/tangram_solver'
