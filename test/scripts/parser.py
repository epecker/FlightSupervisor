import getopt
import os.path
import sys

from classes.MessageParser import MessageParser
from classes.StateParser import StateParser


def main(argv):
    """
    Main method used iterate through directories and create human-readable log files.

    :param argv: List of command line values
    :return: None
    """
    try:
        opts, args = getopt.getopt(argv, "di:o:")
    except getopt.GetoptError:
        print("Files must be named output_state.txt and output_message.txt")
        print("Use -d if it is a test directory where the test directories at labelled using numbers")
        print("parser.py -i <input_directory> -o <output_file>")
        print("parser.py -d -i <input_directory> -o <output_file>")
        sys.exit(2)

    input_dir = ""
    output_file = ""
    test_dir = False
    for opt, arg in opts:
        if opt == "-i":
            input_dir = arg
        elif opt == "-o":
            output_file = arg
        elif opt == "-d":
            test_dir = True
        else:
            print("Bad argument supplied:", opt)
            sys.exit(2)

    with open(output_file, "w") as f:
        if test_dir:
            curr_dir = 0
            while True:
                if os.path.isdir(input_dir + str(curr_dir)):
                    f.write("### Test " + str(curr_dir) + "\n")
                    table = parse_files(input_dir + str(curr_dir))
                    f.write(table.to_string("md"))
                else:
                    break
                curr_dir += 1
        else:
            table = parse_files(input_dir)
            f.write(table.to_string("md"))


def parse_files(input_dir):
    """
    Calls the functions required to parse the states and messages then merges them

    :param input_dir: Directory where the input files are located
    :return: None
    """
    state_parser = StateParser(input_dir + "/output_state.txt")
    message_parser = MessageParser(input_dir + "/output_messages.txt")

    if not state_parser.is_accessible():
        print(input_dir + "/output_state.txt not accessible")
        sys.exit(2)
    if not message_parser.is_accessible():
        print(input_dir + "/output_messages.txt not accessible")
        sys.exit(2)

    state_parser.tidy()
    message_parser.tidy()

    state_table = state_parser.parse()
    message_table = message_parser.parse()

    state_table.outer_join(message_table, "Time")
    state_table.remove_duplicate_rows()
    return state_table


if __name__ == "__main__":
    main(sys.argv[1:])
