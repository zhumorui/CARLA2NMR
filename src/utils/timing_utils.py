import timeit
from functools import wraps

def timed(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        start_time = timeit.default_timer()
        result = func(*args, **kwargs)
        end_time = timeit.default_timer()
        print(f"Function {func.__name__} took {(end_time - start_time):.6f} seconds to run.")
        return result
    return wrapper
