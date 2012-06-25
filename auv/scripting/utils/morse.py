import collections

tab = {
        'A': '.-',      'B': '-...',
        'C': '-.-.',    'D': '-..',
        'E': '.',       'F': '..-.',
        'G': '--.',     'H': '....',
        'I': '..',      'J': '.---',
        'K': '-.-',     'L': '.-..',
        'M': '--',      'N': '-.',
        'O': '---',     'P': '.--.',
        'Q': '--.-',    'R': '.-.',
        'S': '...',     'T': '-',
        'U': '..-',     'V': '...-',
        'W': '.--',     'X': '-..-',
        'Y': '-.--',    'Z': '--..',
        '0': '-----',   ',': '--..--',
        '1': '.----',   '.': '.-.-.-',
        '2': '..---',   '?': '..--..',
        '3': '...--',   ';': '-.-.-.',
        '4': '....-',   ':': '---...',
        '5': '.....',   "'": '.----.',
        '6': '-....',   '-': '-....-',
        '7': '--...',   '/': '-..-.',
        '8': '---..',   '(': '-.--.-',
        '9': '----.',   ')': '-.--.-',
        ' ': ';',       '_': '..--.-',
}

tab = collections.defaultdict(lambda: tab['?'], tab)

def translate(string):
    return ' '.join((tab[c] for c in string.upper()))

def code_length(morse_string, time_map = {';': 3, ' ': 2, '-': 1, '.': 0.5}):
    return sum((time_map[c] for c in morse_string))

