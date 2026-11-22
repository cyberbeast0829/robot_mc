import os
import sys

# 让 `pytest tools/rmc/tests` 无需先 pip install 也能 import rmc
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
