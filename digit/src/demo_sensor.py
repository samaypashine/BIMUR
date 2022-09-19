from digit_interface import Digit
 
d = Digit("D20501") # Unique serial number
d.connect()
d.show_view()
d.disconnect()
