import sys

from .SortedTable import SortedTable

import fileinput
import os
import re
import shutil


class StateParser:
    """
    Used to tidy and parse the state logs created by Cadmium's logging system.

    Attributes
    ----------
    input_log_file: str
        The state log file to tidy and parse.
    inplace: bool
        Determine whether to modify the original file or create a new one.
    """

    def __init__(self, input_log_file, inplace=False):
        """
        Constructs the necessary attributes for the StateParser object.

        :param input_log_file: The state log file to tidy and parse.
        :param inplace: Whether the file should be edited inplace
        """
        self.input_log_file = os.path.abspath(input_log_file)
        self.inplace = inplace

    def tidy(self):
        """
        Tidies the input file for easier parsing.

        :return: None
        """
        tidy_file = self.__get_adjusted_path()
        if not self.inplace:
            shutil.copyfile(self.input_log_file, tidy_file)

        for line in fileinput.FileInput(tidy_file, inplace=True):
            # Adds proper spacing to lines with multiple statements on it.
            # [1:] removes the first \n that we added
            match = re.match("[0-9]{2}:[0-9]{2}:[0-9]{2}:[0-9]{3}", line)
            if not match:
                line = line.replace("State for model", "\nState for model")[1:]
            print(line)

        for line in fileinput.FileInput(tidy_file, inplace=True):
            # Matches the time, prints it with a new line.
            match = re.match("[0-9]{2}:[0-9]{2}:[0-9]{2}:[0-9]{3}", line)
            if match:
                print(match.group(0))
                continue

            # Matches any line not containing "State for model" and removes it.
            match = re.match("^(?:(?!State for model).)*$", line)
            if match:
                continue

            # Matches any line not containing "is State:" and removes it.
            match = re.match("^(?:(?!is State:).)*$", line)
            if match:
                continue

            # Removes "State for model" and "is State"
            line = re.sub("^(State for model \\S*)( \\S* is State)", "\\1", line)
            line = line.replace("is State", "")[16:]
            sys.stdout.write(line)

    def parse(self):
        """
        Parse the tidied input file into a 2D array. The 2d array consists of a
        header with the Time and state names and the rest consists of the data.

        :return: 2D array where the first row is the headers and the rest is the states and time.
        """
        header = []
        first_pass = True
        tidy_file = self.__get_adjusted_path()
        for line in fileinput.FileInput(tidy_file):
            # Matches the time, prints it with a new line.
            match = re.match("[0-9]{2}:[0-9]{2}:[0-9]{2}:[0-9]{3}", line)
            if match:
                if not first_pass:
                    break
                first_pass = False
                continue

            # Append the state name to the header
            header.append(line.split(":")[0].rstrip())
        sorted_table = SortedTable()
        sorted_table.create_header(header, ["Time"])
        sorted_table.add_header_to_table()

        for line in fileinput.FileInput(tidy_file):
            # Matches the time, prints it with a new line.
            match = re.match("[0-9]{2}:[0-9]{2}:[0-9]{2}:[0-9]{3}", line)
            if match:
                sorted_table.next_row()
                sorted_table["Time"] = match.group(0)
                continue
            token = line.split(": ")
            if len(token) != 2:
                print("Not parsable\t\t Time: " + sorted_table[len(sorted_table) - 1][0]
                      + "\t\tValue: " + line.rstrip())
            else:
                sorted_table[token[0].rstrip()] = token[1].rstrip()
        return sorted_table

    def is_accessible(self):
        """
        Verifies that the input file exists and is accessible.

        :return: True if the path and file are accessible false otherwise.
        """

        if self.inplace:
            return os.access(self.input_log_file, os.W_OK)
        else:
            path = os.path.dirname(self.input_log_file)
            return os.access(self.input_log_file, os.R_OK) and os.access(path, os.W_OK)

    def __get_adjusted_path(self):
        """
        Used to get the file path depending on if it is an inplace procedure or not.

        :return: Original path if inplace is false else append .tmp to the path.
        """
        file = self.input_log_file
        if not self.inplace:
            file = file + ".tmp"
        return file
