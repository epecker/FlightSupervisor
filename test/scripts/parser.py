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
        opts, args = getopt.getopt(argv, "adhi:o:")
    except getopt.GetoptError:
        help()
        sys.exit(2)

    if len(opts) == 0:
        help()
        exit(2)

    input_dir = ""
    output_file = ""
    parse_test_cases = False
    parse_test_directories = False
    for opt, arg in opts:
        if opt == "-a":
            parse_test_directories = True
        elif opt == "-i":
            if os.path.isdir(os.path.abspath(arg)):
                input_dir = os.path.abspath(arg) + "/"
            else:
                print("Invalid input path")
                sys.exit(1)
        elif opt == "-o":
            output_file = os.path.abspath(arg)
        elif opt == "-d":
            parse_test_cases = True
        elif opt == "-h":
            help()
        else:
            print("Bad argument supplied:", opt)
            sys.exit(2)

    if input_dir == "":
        print("Input directory is required for all usages")
        sys.exit(1)

    if parse_test_directories:
        test_directories(input_dir)
    elif parse_test_cases:
        if output_file == "":
            print("Output file is required")
            sys.exit(1)

        test_cases(input_dir, output_file)
    else:
        if output_file == "":
            print("Output file is required")
            sys.exit(1)

        with open(output_file, "w") as f:
            table = parse_files(input_dir)
            f.write(table.to_string("md"))


def test_directories(input_dir):
    directories = os.listdir(input_dir)
    directories.sort()
    for d in directories:
        if os.path.isdir(input_dir + d):
            test_cases(input_dir + d + "/", input_dir + d + ".md")


def test_cases(input_dir, output_file):
    """
    Runs through the test cases for a directory

    :param input_dir: Directory where the test cases are located
    :return: None
    """
    with open(output_file, "w") as f:
        curr_dir = 0
        while True:
            if os.path.isdir(input_dir + str(curr_dir)):
                f.write("### Test " + str(curr_dir) + "\n\n")
                table = parse_files(input_dir + str(curr_dir))
                f.write(table.to_string("md"))
                f.write("\n")
            else:
                break
            curr_dir += 1


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


def help():
    print("\nFiles to parse must be named output_state.txt and output_message.txt\n")
    print("-d if it is a test directory where the test directories at labelled using numbers")
    print("-a if it is a directory of test directories")
    print("-h displays this help menu")
    print("-i specify an input directory")
    print("-o specify an output file\n")
    print("python parser.py -i <input_directory> -o <output_file>")
    print("python parser.py -d -i <input_directory> -o <output_file>")
    print("python parser.py -a -i <input_directory>")


if __name__ == "__main__":
    main(sys.argv[1:])
