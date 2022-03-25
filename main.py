from laser_scan import follow
from laser_scan import nav_auto

def main():
    print("Début")
    follow.follow()
    print("appel nav")
    nav_auto.nav_auto()
    print("fin nav")


# if python says run, then we should run
if __name__ == "__main__":
    main()