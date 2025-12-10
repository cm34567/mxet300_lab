from gpiozero.pins.native import NativeFactory
factory = NativeFactory()
print(factory.pin_class(17))

