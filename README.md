
RepRap safety cutoff
Copyright John O'Brien 2016
jweob@cantab.net 2016-07-16

This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>


Turns off the power supply to a RepRap using a relay if any of the following three conditions are met
1. Secondary thermistor reads a temperature above a set threshold
2. Bed heat signal is on for more than a set period of time
3. Hot end is on for more than a set period of time

