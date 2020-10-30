## Copyright (C) 2019 subodh
## 
## This program is free software; you can redistribute it and/or modify it
## under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 3 of the License, or
## (at your option) any later version.
## 
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
## 
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.

## -*- texinfo -*- 
## @deftypefn {} {@var{retval} =} getRotY (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: subodh <subodh@subodh-desktop>
## Created: 2019-10-08

function [R_y] = getRotY(pitch)
R_y = [cos(pitch), 0, sin(pitch);
                0, 1, 0;
      -sin(pitch), 0, cos(pitch)];
end
