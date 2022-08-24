import output_file_parser
import sys
import os

def table_maker(test_path):
    state_table, state_headers = output_file_parser.create_state_table(test_path + "output_state.txt")
    message_table, message_header = output_file_parser.create_message_table(test_path + "output_messages.txt")
    return output_file_parser.create_state_and_message_table(state_table, state_headers, message_table, message_header)

def parse_directory(path):
    if not os.path.isdir(path):
        print("Please run the test drivers first.")
        return 1
    for model in os.listdir(os.fsencode(path)):
        model_name = os.fsdecode(model)
        if path[len(path):len(path)] != "/":
            path = path + "/"
        model_path = path + model_name + "/"
        table = {}
        if os.path.isdir(model_path):
            for test in os.listdir(os.fsencode(model_path)):
                test_name = os.fsdecode(test)
                test_path = model_path + test_name + "/"
                table[int(test_name)] = table_maker(test_path)

            file = open(path + model_name + ".md", "w")
            original_stdout = sys.stdout
            sys.stdout = file
            print("# " + model_name)
            for key, val in sorted(table.items()):
                print("## Test:", key)
                print(val)
                print()
            sys.stdout = original_stdout
            file.close

def main():
    if len(sys.argv) == 2:
        parse_directory(sys.argv[1])
    else: print("Too few or too many arguments")

if __name__ == "__main__":
    main()
