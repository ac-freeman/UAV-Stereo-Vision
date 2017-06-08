import RPi.GPIO as gp

def setupBoard():
    """Enables the correct GPIO pins
    to allow for the Pi to interface
    with up to 4 cameras using the
    Arducam Multi Camera Adapter
    Module."""
    gp.setwarnings(False)
    gp.setmode(gp.BOARD)

    gp.setup(7, gp.OUT)
    gp.setup(11, gp.OUT)
    gp.setup(12, gp.OUT)
     
    gp.setup(15, gp.OUT)
    gp.setup(16, gp.OUT)
    gp.setup(21, gp.OUT)
    gp.setup(22, gp.OUT)

    gp.output(11, True)
    gp.output(12, True)
    gp.output(15, True)
    gp.output(16, True)
    gp.output(21, True)
    gp.output(22, True)

def cameras(cam):
    """Enables the correct GPIO pins to
    enable the correct camera using the
    Arducam Multi Camera Adapter
    Module."""
    
    if cam == "A":
        gp.output(7, False)
        gp.output(11, False)
        gp.output(12, True)
    elif cam == "B":
        gp.output(7, True)
        gp.output(11, False)
        gp.output(12, True)
    elif cam == "C":
        gp.output(7, False)
        gp.output(11, True)
        gp.output(12, False)
    elif cam == "D":
        gp.output(7, True)
        gp.output(11, True)
        gp.output(12, False)
        
    else: raise ValueError, "Need to input A, B, C, or D for cameras."

def finished():
    '''Finished with all the cameras.'''
    
    gp.output(7, False)
    gp.output(11, False)
    gp.output(12, True)
