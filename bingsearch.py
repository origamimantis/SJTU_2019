import urllib.parse
from sys          import argv
from os           import system as term
from pathlib import Path
import datetime


def openblank():
    print('No additional arguments given, opening default webpage.')

    history.write(current_time + ' - Opened default webpage.\n')
    history.write(prevh)
    history.close()
    
    term('firefox')


with open('/home/eric/Documents/bingsearch/history.txt', 'r') as p_h:
    prevh = p_h.read()

history = open('/home/eric/Documents/bingsearch/history.txt', 'w')

current_time = datetime.datetime.now().strftime("%c")

if len(argv) <= 1:
    openblank()

else:
    search = ' '.join( argv[1:] )
    print('Searched for \'%s\'.' % search)
    
    history.write('%s - "%s"\n' % (current_time, search))
    history.write(prevh)
    history.close()
    
    term('firefox https://www.bing.com/search?' + urllib.parse.urlencode( [('q',search )] ) )
print('Program exit.')

