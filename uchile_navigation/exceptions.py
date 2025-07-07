
class NotInitializedError(Exception):
    def __init__(self, msg):
        self.msg = msg
        super().__init__(self.msg) 
    def __str__(self):
        return f'{self.msg}'