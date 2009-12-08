#ifndef __COLORS_H__
#define __COLORS_H__

#include <string>

using namespace std;

const string RED	= "\033[0;31m";
const string GREEN	= "\033[0;32m";
const string GRAY   = "\033[0m";
const string BLUE	= "\033[34m";

const string BOLD	= "\033[1m";

/*
Black       0;30     Dark Gray     1;30
Blue        0;34     Light Blue    1;34
Green       0;32     Light Green   1;32
Cyan        0;36     Light Cyan    1;36
Red         0;31     Light Red     1;31
Purple      0;35     Light Purple  1;35
Brown       0;33     Yellow        1;33
Light Gray  0;37     White         1;37
*/

/*
\033[22;30m - black
\033[22;31m - red
\033[22;32m - green
\033[22;33m - brown
\033[22;34m - blue
\033[22;35m - magenta
\033[22;36m - cyan
\033[22;37m - gray
\033[01;30m - dark gray
\033[01;31m - light red
\033[01;32m - light green
\033[01;33m - yellow
\033[01;34m - light blue
\033[01;35m - light magenta
\033[01;36m - light cyan
\033[01;37m - white
*/

/*
\\033[30m set foreground color to black
\\033[31m set foreground color to red
\\033[32m set foreground color to green
\\033[33m set foreground color to yellow
\\033[34m set foreground color to blue
\\033[35m set foreground color to magenta (purple)
\\033[36m set foreground color to cyan
\\033[37m set foreground color to white
\\033[40m set background color to black
\\033[41m set background color to red
\\033[42m set background color to green
\\033[43m set background color to yellow
\\033[44m set background color to blue
\\033[45m set background color to magenta (purple)
\\033[46m set background color to cyan
\\033[47m set background color to white
\\033[1;30m set foreground color to dark gray
\\033[1;31m set foreground color to light red
\\033[1;32m set foreground color to light green
\\033[1;33m set foreground color to yellow
\\033[1;34m set foreground color to light blue
\\033[1;35m set foreground color to light magenta (purple)
\\033[1;36m set foreground color to light cyan
\\033[1;37m set foreground color to white
\\033[1;40m set background color to dark gray
\\033[1;41m set background color to light red
\\033[1;42m set background color to light green
\\033[1;43m set background color to yellow
\\033[1;44m set background color to light blue
\\033[1;45m set background color to light magenta (purple)
\\033[1;46m set background color to light cyan
\\033[1;47m set background color to white

For other features:
\\033[0m reset; clears all colors and styles (to white on black)
\\033[1m bold on
\\033[3m italics on
\\033[4m underline on
\\033[5m blink on
\\033[7m reverse video on
\\033[8m nondisplayed (invisible)
\\033[x;yH moves cursor to line x, column y
\\033[xA moves cursor up x lines
\\033[xB moves cursor down x lines
\\033[xC moves cursor right x spaces
\\033[xD moves cursor left x spaces
\\033[2J clear screen and home cursor
*/
#endif

