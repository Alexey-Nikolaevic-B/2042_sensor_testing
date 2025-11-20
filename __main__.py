import traceback

from src.core import Core
from src.console import run

if __name__ == '__main__':
    core = Core()

    try:
        run(core)
    except Exception as e:
        print(f"\n❌ Erorr while running.")
        print(f"\nDetails:")
        traceback.print_exc()
        
    finally:
        core.kill()