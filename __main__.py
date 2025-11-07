import traceback
from src.app import App

if __name__ == '__main__':
    config = 1        
    console = App(config)
    
    try:
        console.run()
    except Exception as e:
        print(f"\n❌ Erorr while running.")
        print(f"\nDetails:")
        traceback.print_exc()
        
    finally:
        console.exit()

