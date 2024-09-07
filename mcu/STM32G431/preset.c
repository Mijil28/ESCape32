/*
** Copyright (C) Arseny Vakhrushev <arseny.vakhrushev@me.com>
**
** This firmware is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** This firmware is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this firmware. If not, see <http://www.gnu.org/licenses/>.
*/

#include "common.h"

const uint16_t sinedata[] = {
	 560,  570,  580,  589,  599,  609,  619,  628,  638,  648,  657,  667,  676,  686,  695,
	 705,  714,  724,  733,  742,  752,  761,  770,  779,  788,  797,  805,  814,  823,  831,
	 840,  848,  857,  865,  873,  881,  889,  897,  905,  912,  920,  927,  935,  942,  949,
	 956,  963,  970,  976,  983,  989,  995, 1001, 1007, 1013, 1019, 1024, 1030, 1035, 1040,
	1045, 1050, 1054, 1059, 1063, 1068, 1072, 1075, 1079, 1083, 1086, 1089, 1093, 1096, 1098,
	1101, 1103, 1106, 1108, 1110, 1111, 1113, 1115, 1116, 1117, 1118, 1119, 1119, 1120, 1120,
	1120, 1120, 1120, 1119, 1119, 1118, 1117, 1116, 1115, 1113, 1111, 1110, 1108, 1106, 1103,
	1101, 1098, 1096, 1093, 1089, 1086, 1083, 1079, 1075, 1072, 1068, 1063, 1059, 1054, 1050,
	1045, 1040, 1035, 1030, 1024, 1019, 1013, 1007, 1001,  995,  989,  983,  976,  970,  963,
	 956,  949,  942,  935,  927,  920,  912,  905,  897,  889,  881,  873,  865,  857,  848,
	 840,  831,  823,  814,  805,  797,  788,  779,  770,  761,  752,  742,  733,  724,  714,
	 705,  695,  686,  676,  667,  657,  648,  638,  628,  619,  609,  599,  589,  580,  570,
	 560,  550,  540,  531,  521,  511,  501,  492,  482,  472,  463,  453,  444,  434,  425,
	 415,  406,  396,  387,  378,  368,  359,  350,  341,  332,  323,  315,  306,  297,  289,
	 280,  272,  263,  255,  247,  239,  231,  223,  215,  208,  200,  193,  185,  178,  171,
	 164,  157,  150,  144,  137,  131,  125,  119,  113,  107,  101,   96,   90,   85,   80,
	  75,   70,   66,   61,   57,   52,   48,   45,   41,   37,   34,   31,   27,   24,   22,
	  19,   17,   14,   12,   10,    9,    7,    5,    4,    3,    2,    1,    1,    0,    0,
	   0,    0,    0,    1,    1,    2,    3,    4,    5,    7,    9,   10,   12,   14,   17,
	  19,   22,   24,   27,   31,   34,   37,   41,   45,   48,   52,   57,   61,   66,   70,
	  75,   80,   85,   90,   96,  101,  107,  113,  119,  125,  131,  137,  144,  150,  157,
	 164,  171,  178,  185,  193,  200,  208,  215,  223,  231,  239,  247,  255,  263,  272,
	 280,  289,  297,  306,  315,  323,  332,  341,  350,  359,  368,  378,  387,  396,  406,
	 415,  425,  434,  444,  453,  463,  472,  482,  492,  501,  511,  521,  531,  540,  550,
};
