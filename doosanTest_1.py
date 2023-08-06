from DoosanV1 import Doosan

if __name__ == "__main__":
    ds = Doosan("192.168.5.100")
    ds.Initialize(mode='terminal', filepath="./pick_place.txt")
    ds.Disconnect()
