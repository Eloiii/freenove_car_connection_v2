if __name__ == '__main__':
    rgbcode = 16711935
    print(f'{(((rgbcode & 0x00FF0000) >> 16), ((rgbcode & 0x00FF00) >> 8), (rgbcode & 0x0000FF))}')
