#!/usr/bin/env python3
#
# Copyright 2019 Mateusz Polnik
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import re


class MiniZincLoader:
    INT_PATTERN = re.compile('^\d+$')
    FLOAT_PATTERN = re.compile('^\d+\.\d+$')
    STRING_PATTERN = re.compile('^"(.*?)"$')
    ARRAY_PATTERN = re.compile('^\[([^\]]*?)\]$')
    ENUM_SET_PATTERN = re.compile('^{(.*?)}$')
    DECLARATION_PATTERN = re.compile('(\w[\w_\d]*)\s*=\s*(.*?);')

    def load(self, file_pointer):
        variables = {}

        file_content = ''.join([line.strip() for line in file_pointer])
        for variable, raw_value in MiniZincLoader.DECLARATION_PATTERN.findall(file_content):
            variables[variable] = self.__parse_mini_zinc_variable(raw_value)
        return variables

    def __parse_mini_zinc_variable(self, raw_value):
        match = MiniZincLoader.INT_PATTERN.match(raw_value)
        if match:
            return int(raw_value)

        match = MiniZincLoader.FLOAT_PATTERN.match(raw_value)
        if match:
            return float(raw_value)

        match = MiniZincLoader.STRING_PATTERN.match(raw_value)
        if match:
            return match.group(1)

        match = MiniZincLoader.ENUM_SET_PATTERN.match(raw_value)
        if match:
            return self.__parse_array_row(match.group(1))

        match = MiniZincLoader.ARRAY_PATTERN.match(raw_value)
        if match:
            return self.__parse_array(match.group(1))

        return raw_value

    def __parse_array_row(self, value):
        elements = value.split(',')
        return [self.__parse_mini_zinc_variable(element.strip()) for element in elements]

    def __parse_array(self, raw_value):
        raw_rows = raw_value.split('|')
        if len(raw_rows) == 1:
            return self.__parse_array_row(raw_rows[0])

        rows = []
        for raw_row in raw_rows:
            if not raw_row:
                continue

            row = self.__parse_array_row(raw_row)
            if row:
                rows.append(row)
        return rows
