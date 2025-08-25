import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/didi_121/Rescue-Major/workspace/GUI/install/GUI'
