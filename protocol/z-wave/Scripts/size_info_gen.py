import string
import sys
import getopt

def main(argv):
     input_file = ""
     printEnabled = False
     
     try:
       opts, args = getopt.getopt(argv,"phi:",["ifile="])
     except getopt.GetoptError:
       print("size_info_gen.py -i <inputfile>")
       sys.exit(2)

     for opt, arg in opts:
       if opt == '-h':
         print("size_info_gen.py -i <inputfile>")
         sys.exit()
       elif opt == '-p':
         printEnabled = True
       elif opt in ("-i", "--ifile"):
         input_file = arg

     # Only print the input file, if the printing of the results are disabled.
     if printEnabled == False:
       print("input file for code size info is: %s" % input_file)

     f= open(input_file,"r+")
     contents = f.read()
     contents = contents.rstrip("\n\r")
     input_lines = contents.splitlines()
     
     # Initialize counters
     flash_binary_size = 0
     flash_storage_size = 0
     sram_size = 0

     # Do the summation line-by-line
     for line in input_lines:
       cols = line.split()
       if cols[0] in ['.text', '_cc_handlers', '.ARM.exidx','.data']:
         flash_binary_size = flash_binary_size + int(cols[1], 16)
       elif cols[0] in ['.nvm3App', '.simee']:
         flash_storage_size = flash_storage_size + int(cols[1], 16)
       elif cols[0] in ['.data', '.bss', '.heap', '.stack_dummy', '.reset_info']:
         sram_size = sram_size + int(cols[1], 16)

     # Write to the output file: (This modifies the input file)
     f.seek(0)
     f.write("\n")
     f.write("==========================================================\n")
     f.write("The output of the size tool: (e.g. arm-none-ambi-size.exe)\n")
     f.write("==========================================================\n")
     f.write("\n")
     f.write(contents)
     f.write("\n")
     f.write("\n")
     f.write("The calculated FLASH and SRAM usage summary:\n")
     f.write("============================================\n")
     f.write("FLASH used for binary:  (Including only the sections: .text, _cc_handlers, .ARM.exidx, .data)\n")
     f.write("   " + str(flash_binary_size) + "\n")
     f.write("FLASH uses for storage: (Including only the sections: .nvm3App, .simee)\n")
     f.write("   " + str(flash_storage_size) + "\n")
     f.write("SRAM usage:             (Including only the sections: .data, .bss, .heap, .stack_dummy, .reset_info)\n")
     f.write("   " + str(sram_size) + "\n")
     f.write("\n")

     # Print generated file content
     if printEnabled == True:
       f.seek(0)
       contents = f.read()
       print(contents)

     f.close()
if __name__== "__main__":
  main(sys.argv[1:])