import sys
from PyQt5.QtWidgets import QApplication

from ui_main import Main_UI        
 
if __name__ == "__main__":
    mainApp = QApplication(sys.argv)
    app = Main_UI()
    app.show()

    try:
        sys.exit(mainApp.exec_())
    except:
        print('Exiting')