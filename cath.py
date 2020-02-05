
AMOUNT_TO_SHOW = 15

s = open("/home/eric/Documents/bingsearch/history.txt", 'r')

list_o_hist_o = []

try: int(AMOUNT_TO_SHOW)
except ValueError: AMOUNT_TO_SHOW = 15

if AMOUNT_TO_SHOW <= 0: AMOUNT_TO_SHOW = 15

for x in range(AMOUNT_TO_SHOW):
    
    try: list_o_hist_o.append(next(s))

    except StopIteration: break

sep = '------------------------------------------------'
print(sep)
print(" Showing the newest %d searches attempted using\n googsearch.py; Change this by editing constant\n'AMOUNT_TO_SHOW' in ~/Documents/bingsearch/cath.py" %  AMOUNT_TO_SHOW)
print(sep)
print('\n' + ''.join(reversed(list_o_hist_o))) if len(list_o_hist_o) else print("\nThere's nothing here...\n")
print(sep)
