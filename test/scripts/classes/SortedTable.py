class SortedTable:
    """
    Used create sorted 2D arrays.

    Attributes
    ----------
    table: [[]]
        2D array.
    header: {}
        header for the 2D array.
    current_row: int
        specifies the current row to add to.
    """

    def __init__(self):
        """
        Constructs the necessary attributes for the SortedTable object.
        """
        self.table = [[]]
        self.header = {}
        self.current_row = 0

    def create_header(self, sorted_header=None, pre_header=None, post_header=None):
        """
        Creates the header that is used for sorting the values into the array.

        :param sorted_header: Values to be sorted in the header.
        :param pre_header: Values to place as is into the front of the header.
        :param post_header: Values to place as is into the back of the header
        :return: None
        """

        # Adds headers to the beginning unsorted.
        if pre_header:
            for i, val in enumerate(pre_header):
                self.header[val] = i

        # Adds headers after sorting.
        start_index = len(self.header)
        if sorted_header:
            sorted_header.sort()
            for i, val in enumerate(sorted_header):
                self.header[val] = i + start_index

        # Adds headers to the end unsorted.
        start_index = len(self.header)
        if post_header:
            for i, val in enumerate(post_header):
                self.header[val] = i + start_index

    def add_header_to_table(self):
        """
        Adds the header to the 2D list.

        :return: None
        """
        if self.header:
            if len(self.table) > 0:
                self.table[0] = self.get_header()
            else:
                self.table.insert(0, self.get_header())

    def get_header(self):
        """
        Creates a list of sorted headers.

        :return: List of strings representing the sorted headers.
        """
        if self.header:
            header = [""]*len(self.header)
            for key in self.header:
                header[self.header[key]] = key
            return header

    def get_table(self):
        """
        Return the attribute table.

        :return: Table of values.
        """
        return self.table

    def next_row(self):
        """
        Allocates space for the next row.

        :return: None
        """
        self.table.append([""] * len(self.header))
        self.current_row = self.current_row + 1

    def outer_join(self, table, where_equal):
        """
        Joins to sorted tables together. Not a robust solution

        :param table: SortedTable to join
        :param where_equal: Column to match when performing the join
        :return: None
        """
        self_match_index = self.table[0].index(where_equal)
        table_match_index = table[0].index(where_equal)

        header = table[0][1:]
        self.create_header(post_header=header)
        self.add_header_to_table()

        inserted = ""
        table_map_num = {}
        table_map_index = {}
        for i, val in enumerate(table[1:]):
            if val[table_match_index] != "":
                inserted = val[table_match_index]
                table_map_num[inserted] = 0
                table_map_index[inserted] = i + 1
            else:
                table_map_num[inserted] += 1

        for i in range(len(self) - 1, 0, -1):
            row = self[i]
            row.extend([""] * (len(table[0]) - 1))
            if row[self_match_index] in table_map_index:
                table_index = table_map_index[row[self_match_index]]
                for j, val in enumerate(table[table_index][1:]):
                    self.replace(header[j], val, i)

                output_num = table_map_num[row[self_match_index]]
                for j in range(1, output_num + 1):
                    self.insert_row_at(i + j)
                    for k, val in enumerate(table[table_index + j][1:]):
                        self.replace(header[k], val, i + j)

    def to_string(self, output_format="csv"):
        """
        Generates a string of the table attribute in either the
        csv or Markdown format.

        :param output_format: Currently supports either "csv" or "md" for markdown.
        :return: String representation of the table attribute.
        """
        if output_format == "csv":
            return "".join(self.__csv())
        elif output_format == "md":
            return "".join(self.__markdown())
        return ""

    def replace(self, key, value, row):
        index = self.header.get(key)
        self.table[row][index] = value

    def insert_row_at(self, row):
        self.table.insert(row, [""] * len(self.header))

    def remove_duplicate_rows(self):
        for i in range(len(self.table) - 1, 0, -1):
            if self.table[i][0] == "":
                continue

            is_same = True
            curr_row = self.table[i]
            next_row = self.table[i - 1]
            for j in range(2, len(self.table[0])):
                if curr_row[j] != next_row[j]:
                    is_same = False
                    break

            if is_same:
                self.table.pop(i)

    def __csv(self):
        """
        Creates list of the table attribute in the cvs format.

        :return: List containing the strings of the cvs format output.
        """
        string_list = []

        for row in self.table:
            for val in row:
                string_list.append(val)
                string_list.append(", ")
            string_list.pop()
            string_list.append("\n")

        return string_list

    def __markdown(self):
        """
        Creates list of the table attribute in the Markdown format.

        :return: List containing the strings of the Markdown format output.
        """
        string_list = []
        largest_string = [0] * len(self.header)

        for row in self.table:
            for i, val in enumerate(row):
                if len(val) > largest_string[i]:
                    largest_string[i] = len(val)

        for i, row in enumerate(self.table):
            string_list.append("|")

            for j, val in enumerate(row):
                string_list.append(" ")
                string_list.append(val)

                if len(val) < largest_string[j]:
                    dif = largest_string[j] - len(val)
                    string_list.append(" " * dif)
                string_list.append(" |")
            string_list.append("\n")

            if i == 0:
                string_list.append("|")

                for val in largest_string:
                    string_list.append("-" * (val + 2))
                    string_list.append("|")
                string_list.append("\n")
        return string_list

    def __getitem__(self, index):
        return self.table[index]

    def __setitem__(self, key, value):
        """
        Adds a value to the 2D array in the current row denoted by current_row.

        :param key: Name of the heading the value will be placed under.
        :param value: Value to insert under the given header.
        :return: None
        """
        index = self.header.get(key)
        if index is not None:
            self.table[self.current_row][index] = value
        else:
            print("Key does not exist\t Time: " + self.table[self.current_row][0] + "\t\tValue: " + key)

    def __len__(self):
        return len(self.table)

    def __iter__(self):
        """
        Also the table nested in the class to be iterated over.

        :return: Table that can be iterated over.
        """
        return iter(self.table)
