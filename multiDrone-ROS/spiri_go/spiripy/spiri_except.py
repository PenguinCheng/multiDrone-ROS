class SpiriGoConnectionError(Exception):
    def __init__(self, description = "unknown"):
        self.description = description
