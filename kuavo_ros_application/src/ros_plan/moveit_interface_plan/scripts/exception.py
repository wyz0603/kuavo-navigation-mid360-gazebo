from threading import ThreadError
from queue import Empty

class MoveitInitError(Exception):
    ...
    
class PublishTnreadError(ThreadError):
    ...

class GetEmptyTraj(Empty):
    ...
