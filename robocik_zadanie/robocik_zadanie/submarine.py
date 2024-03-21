import pygame

class Submarine:
    def __init__(self, game):
        self.screen = game.screen
        self.screen_rect = game.screen.get_rect()
        self.settings = game.settings

        self.image_right = pygame.image.load("src/robocik_zadanie/resource/submarine_right.bmp")
        self.image_left = pygame.image.load("src/robocik_zadanie/resource/submarine_left.bmp")

        self.image = self.image_right
        self.rect = self.image.get_rect()

        self.rect.midbottom = self.screen_rect.midbottom
    
    def blitme(self):
        self.screen.blit(self.image, self.rect) 