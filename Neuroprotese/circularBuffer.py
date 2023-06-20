from collections import deque

class CircularBuffer:
    def __init__(self, capacity):
        self.buffer = deque(maxlen=capacity)

    def push(self, item):
        self.buffer.append(item)

    def get(self):
        return list(self.buffer)
    
    def get_oldest(self):
        if len(self.buffer) > 0:  # Verifica se o buffer não está vazio
            return self.buffer.popleft()
        else:
            return None  # Ou outra ação apropriada para lidar com o buffer vazio