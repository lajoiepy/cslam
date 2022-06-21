""""
This file contains various utilatary functions 
for basic number and list operations
"""

def clamp(num, min_value, max_value):
   return max(min(num, max_value), min_value)

def list_clamp(l, idx):
   i = clamp(idx, 0, len(l)-1)
   return l[i]

def list_range(l, start):
   s = clamp(start, 0, len(l)-1)
   return [l[i] for i in range(s, len(l)-1)]

def list_chunks(l, start, chunk_size):
   s = clamp(start, 0, len(l)-1)
   return [l[i:i+chunk_size] for i in range(s, len(l), chunk_size)]
