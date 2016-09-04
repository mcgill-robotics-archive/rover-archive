

## Change the background of an object to red
#
# @param thing An object implementing the QWidget Interface. This function uses
# the setStyleSheet function of the interface.
def lbl_bg_red(thing, text):
    """sets a style sheet to the @param thing resulting in a red background"""
    thing.setStyleSheet('background-color:#ff0000')
    thing.setText(text)


## Change the background of an object to the default light gray background color
#
# @param thing An object implementing the QWidget Interface. This function uses
# the setStyleSheet function of the interface.
def lbl_bg_grn(thing, text):
    """sets a style sheet to the @param thing resulting in a green background"""
    thing.setStyleSheet('background-color:#33CC33')
    thing.setText(text)
