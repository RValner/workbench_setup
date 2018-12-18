import sys

in_string = str(sys.argv[1])

def escape_special_chars(in_string):
    out_string = ""

    for c in in_string:
        if not c.isalnum():
            out_string += "\\" + c
        else:
            out_string += c

    return out_string

print escape_special_chars(in_string)