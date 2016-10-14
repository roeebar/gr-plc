#!/bin/bash
#
# Gr-plc - IEEE 1901 module for GNU Radio
# Copyright (C) 2016 Roee Bar <roeeb@ece.ubc.ca>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

echo '#define IEEE1901_CHANNELS_BROADCAST_MASK ( \'
printf '0,%0.s' {0..70} && printf ' \\\n'
printf '0,%0.s' {71..73} && printf ' \\\n'
printf '0,%0.s' {74..85} && printf ' \\\n'
printf '1,%0.s' {86..139} && printf ' \\\n'
printf '0,%0.s' {140..167} && printf ' \\\n'
printf '1,%0.s' {168..214} && printf ' \\\n'
printf '0,%0.s' {215..225} && printf ' \\\n'
printf '1,%0.s' {226..282} && printf ' \\\n'
printf '0,%0.s' {283..302} && printf ' \\\n'
printf '1,%0.s' {303..409} && printf ' \\\n'
printf '0,%0.s' {410..419} && printf ' \\\n'
printf '1,%0.s' {420..569} && printf ' \\\n'
printf '0,%0.s' {570..591} && printf ' \\\n'
printf '1,%0.s' {592..736} && printf ' \\\n'
printf '0,%0.s' {737..748} && printf ' \\\n'
printf '1,%0.s' {749..856} && printf ' \\\n'
printf '0,%0.s' {857..882} && printf ' \\\n'
printf '1,%0.s' {883..1015} && printf ' \\\n'
printf '0,%0.s' {1016..1027} && printf ' \\\n'
printf '1,%0.s' {1028..1143} && printf ' \\\n'
printf '0,%0.s' {1144..1220} && printf ' \\\n'
printf '0,%0.s' {1221..1224} && printf ' \\\n'
printf '0,%0.s' {1225..2046}
printf '0' && printf ' \\\n'
echo ')'
