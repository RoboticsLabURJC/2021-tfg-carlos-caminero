import follow_person.HAL as HAL


# ----- USER PROGRAM -----

v = 0.3
HAL.setV(v)
HAL.setW(0)

def user_main(args=None):
    global v
    v += 0.001
    HAL.setV(v)

##########################


# ----- DO NOT TOUCH -----
def main():
    HAL.main(user_main)

if __name__ == '__main__':
    main()