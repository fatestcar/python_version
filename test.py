# import ctypes
# so = ctypes.CDLL('great_module.so')
# print(dir(so))
# # so.great_functions(2)

import ctypes
ll = ctypes.cdll.LoadLibrary
lib = ll("./test.so")
lib.init()

#
# great_function(2)
