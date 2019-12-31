import serial
import repowered
import serial.tools.list_ports

# CMD_READ_CONFIG	= 0x11,
# CMD_READ_ANCHORS 	= 0x12,
# CMD_SET_CONFIG 	= 0x22,
# CMD_RANGE 		= 0x33,
# CMD_RESTART 		= 0x44,
# CMD_RESET 		= 0x55,
# CMD_SAVE_CONFIG 	= 0x66
cmd_com  = "com"
cmd_read = "read"
cmd_set  = "set"
cmd_save = "save"
cmd_range = "range"
cmd_quit = "quit"
cmd_help = "help"

opt_id_v   = "--id"
opt_mode_v = "--mode"

arg_tag    = "tag"
arg_anchor = "anchor"

options =  cmd_com + "   : Set comport\r\n"
options += cmd_read + "  : Read configuration\n\r"
options += cmd_set + "   : Set configuration\n\r"
options +=        "      :   " + opt_id_v + "=[N]\n\r"
options +=        "      :   " + opt_mode_v + "=["+ arg_tag + ", " + arg_anchor + "]\n\r"
options += cmd_range + " : trigger a ranging transaction\n\r"
options += cmd_save + "  : save configuration\n\r"
options += cmd_help + "  : display this menu\n\r"
options += cmd_quit + "  : quit\r\n"


ports = serial.tools.list_ports.comports()

ser = serial.Serial()
def main():
    print("Available Comports")
    for port in ports:
        print(port.name)

    print("-----")

    print(options)

    uwb = None


    done = False
    while not done:
        s = input("> ")
        words = s.split(' ')

        if words[0].lower() == cmd_com:
            uwb = repowered.UwbConfig(words[1])
            uwb.id = 0
            uwb.mode = 0
            uwb.channel = 2
            uwb.samples_per_range = 1
            uwb.number_of_anchors = 5

        elif words[0].lower() == cmd_read:
            print("reading...")
            print(uwb.read_config())
            
        elif words[0].lower() == cmd_set:
            print("setting...")
            print(uwb.set_config())
        elif words[0].lower() == cmd_save:
            print("saving...")
        elif words[0].lower() == cmd_help:
            print(options)
        elif words[0].lower() == cmd_quit:
            print("quitting...")
            if ser.is_open:
                ser.close()
            done = True


if __name__ == '__main__':
    main()