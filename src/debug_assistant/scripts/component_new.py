from surfaces   import *
from algorithms import *
from input import input_manager

# from toros import pubSDFLayerVis

SELECTOR_SIZE = 50

SEL_CLICK_NONE = 0
SEL_CLICK_LEFT = 1
SEL_CLICK_RIGHT = 2
SEL_CLICK_MID = 3

class BasicComponent:
    def __init__(self):
        
        

component_manager = ComponentManager()
component_manager.initComponents()

news_component_manager = ComponentManager()
news_component_manager.components.append( StaticTitle((35,20), "Logs", COLOR_YELLOW))



