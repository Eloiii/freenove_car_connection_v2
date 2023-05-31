import psutil

if __name__ == '__main__':
    print(len(list(psutil.process_iter())))
