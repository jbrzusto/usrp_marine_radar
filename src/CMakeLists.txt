#
# Copyright 2010-2011 Ettus Research LLC
# Copyright 2011 John Brzustowski
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

########################################################################
# usrp marine radar digitizer host side code
########################################################################
SET(marine_radar_sources
)

#for each source: build an executable and install
FOREACH(marine_radar_source ${marine_radar_sources})
    GET_FILENAME_COMPONENT(example_name ${marine_radar_source} NAME_WE)
    ADD_EXECUTABLE(${example_name} ${marine_radar_source})
    TARGET_LINK_LIBRARIES(${example_name} uhd)
    INSTALL(TARGETS ${example_name} RUNTIME DESTINATION ${PKG_DATA_DIR}/examples COMPONENT examples)
ENDFOREACH(marine_radar_source)

