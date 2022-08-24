import fileinput
from tabulate import tabulate
import copy
import re

# Cleans up the output message file to prepare it for parsing.
# This is done in place and will overwrite the original file
def cleanup_message_file(file_path):
    for line in fileinput.FileInput(file_path, inplace=1):
        match = re.match("^\n", line)
        if match:
            continue
        match = re.findall("[a-zA-Z]\[s", line)
        if match:
            for i, val in enumerate(match):
                new_string = val[:1]+"\n"+val[1:]
                line=line.replace(val, new_string)
        print(line, end = '')

# Cleans up the state message file to prepare it for parsing.
# This is done in place and will overwrite the original file
def cleanup_state_file(file_path):
    for line in fileinput.FileInput(file_path, inplace=1):
        match = re.match("^\n", line)
        if match:
            continue
        match = re.findall("[a-z]S", line)
        if match:
            for i, val in enumerate(match):
                new_string = val[:1]+"\n"+val[1:]
                line=line.replace(val, new_string)
        match = re.findall("[0-9]S", line)
        if match:
            for i, val in enumerate(match):
                new_string = val[:1]+"\n"+val[1:]
                line=line.replace(val, new_string)
        print(line, end = '')

# Parses a output message file to create a dom style structure
def create_message_tree(file_path):
    cleanup_message_file(file_path)
    file = open(file_path, "r")
    tree = []

    current_line = file.readline()
    while len(current_line) != 0:
        match = re.match("[0-9][0-9]:[0-9][0-9]:[0-9][0-9]:[0-9][0-9][0-9]", current_line)
        if match:
            tree.append([match.group(), {}])
            current_line = file.readline()
            continue
        match = re.search("::([^}{,\s]+):\s{(?!\s*})([^}{]+)}", current_line)
        if match:
            message = match.group(1)
            temp = message.split(">::")
            if len(temp) == 2:
                message = temp[1]
            value = match.group(2)
            tokens = current_line.split()
            model = tokens[len(tokens)-1]
            tree[len(tree)-1][1][model]=[message, value]
        current_line = file.readline()
    return tree

# Parses a state output file to create a dom style structure
def create_state_tree(file_path):
    cleanup_state_file(file_path)
    file = open(file_path, "r")
    tree = []

    current_line = file.readline()
    while len(current_line) != 0:
        match = re.match("[0-9][0-9]:[0-9][0-9]:[0-9][0-9]:[0-9][0-9][0-9]", current_line)
        if match:
            tree.append([match.group(), {}])
            current_line = file.readline()
            continue
        match = re.search(" IR_| ir_", current_line)
        if match:
            current_line = file.readline()
            continue
        match = re.search("\s[a-zA-Z_0-9]+\sis\sState:\s[a-zA-Z_\-0-9]+\n", current_line)
        if match:
                tokens = current_line.split()
                model = tokens[3]
                model_state = tokens[len(tokens)-1]
                tree[len(tree)-1][1][model]=model_state
        current_line = file.readline()
    return tree

# Creates a list from a message output file sorted by time
def create_message_table(file_path):
    tree = create_message_tree(file_path)
    if len(tree) == 0:
        return

    table = []
    header = ["Time", "Output Model", "Output Port", "Value"]

    i = 0
    for index, message in enumerate(tree):
        for key, value in sorted(message[1].items()):
            table.append([message[0]])
            table[i].append(key)
            table[i].append(value[0])
            table[i].append(value[1])
            i = i + 1
    return table, header

# Creates a list from a state output file sorted by time
def create_state_table(file_path):
    tree = create_state_tree(file_path)
    if len(tree) == 0:
        return

    header = []
    header.append("Time")
    for key, value in sorted(tree[0][1].items()):
        header.append(key)
    table = []
    for i, state in enumerate(tree):
        table.append([state[0]])
        table[i] = table[i] + [""]*(len(header)-1)
        for key, value in sorted(state[1].items()):
            for j in range(1, len(header)):
                if key == header[j]:
                    table[i][j] = value
                    break
    for i in range(1, len(header)):
        header[i] = header[i] + " State"
    return table, header

# Combines the state table and the message table.
def create_state_and_message_table(state_table, state_headers, message_table, message_header):
    header = copy.copy(state_headers)
    header.extend(message_header[1:len(message_header)])

    pad = len(state_headers) - 1
    padded_message_table = []
    for i, value in enumerate(message_table):
        padded_message_table.append([value[0]] + [''] * pad)
        padded_message_table[i].extend(value[1:])

    pad = len(message_header) - 1
    padded_state_table = []
    for i, value in enumerate(state_table):
        padded_state_table.append(value)
        padded_state_table[i].extend([''] * pad)

    table = padded_state_table
    table.extend(padded_message_table)
    table.sort(key=lambda list: list[0])

    items_to_remove = []
    current_foundation = 0
    for i, value in enumerate(table):
        if i == 0: continue
        if table[current_foundation][1:len(value)-1] == value[1:len(value)-1]:
            items_to_remove.append(i)
        else:
            is_line_appended = False
            part = value[1:len(value)-3]
            for j in part:
                if j == "" and value[len(value)-2] == "":
                    items_to_remove.append(i)
                    is_line_appended = True
                    break
            if not is_line_appended:
                current_foundation = i
    for i in reversed(items_to_remove):
        table.pop(i)

    viewed_times = {}
    for i, value in enumerate(table):
        if i == 0: continue
        if i >= len(table)-1: break
        if value[1] != '' and table[i+1][1] == '' and (value[0] == table[i+1][0]):
            row1 = copy.copy(value[:len(header)-3])
            row2 = copy.copy(table[i+1][len(header)-3:])
            row1.extend(row2)
            table[i] = row1
            table.pop(i+1)
        if viewed_times.get(value[0]):
            table[i][0] = ''
        else:
            viewed_times[value[0]] = True
    return tabulate(table, headers=header, tablefmt="github")

## Example Usage
# state_table, state_headers = create_state_table("test/simulation_results/lp_reposition/0/output_state.txt")
# message_table, message_header = create_message_table("test/simulation_results/lp_reposition/0/output_messages.txt")
# print(create_state_and_message_table(state_table, state_headers, message_table, message_header))
