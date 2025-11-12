import traceback

from src.core import Core
from src.console import ConsoleUi

if __name__ == '__main__':
    core = Core()
    ui = ConsoleUi(core)

    try:
        ui.run()
    except Exception as e:
        print(f"\n❌ Erorr while running.")
        print(f"\nDetails:")
        traceback.print_exc()
        
    finally:
        core.kill()