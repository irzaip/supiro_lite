#!/usr/bin/env python

import pygame

import rospy
from supiro_lite.msg import motorpower
from supiro_lite.msg import encoder


def sendpower(leftpwr, rightpwr, leftdir, rightdir):
	mtrpwr = motorpower()
	mtrpwr.leftpwr = leftpwr
	mtrpwr.rightpwr = rightpwr
	mtrpwr.leftdir = leftdir
	mtrpwr.rightdir = rightdir
	pub.publish(mtrpwr)

def callback(data):
	print("leftenc:", data.leftenc, " rightenc:", data.rightenc)

def displaytext(screen):

    basicfont = pygame.font.Font("monos.ttf", 90)
    lvfont = pygame.font.Font("monos.ttf", 30)
    lv = lvfont.render("1000", True, (0,0,0))

    images = []
    

    text = basicfont.render("Hello", True, (0,0,0))
    images.append(text)
    textrect = text.get_rect()
    textrect.centerx = screen.get_rect().centerx
    textrect.centery = screen.get_rect().centery

    screen.fill((255, 255, 255))
    for i in images:
        screen.blit(i, textrect)
    texx = lv.get_rect()
    screen.blit(lv, texx)

    pygame.display.update()

def teleop():

    pygame.init()
    pygame.font.init()
    pygame.display.set_caption("Baca Saya")
    screen = pygame.display.set_mode((1080,700))

    #app1 = pygame.mixer.Sound("applause.wav")


    #formdata()

    # define a variable to control the main loop
    #running = True
     
    clock = pygame.time.Clock()
    
    displaytext(screen)

    pub = rospy.Publisher("motorpwr", motorpower)
    rospy.init_node("pwr", anonymous=True)
    rate = rospy.Rate(2)
    
    rospy.Subscriber('encoder', encoder, callback)


    while not rospy.is_shutdown():

        for event in pygame.event.get():
            # only do something if the event is of type QUIT
            if event.type == pygame.QUIT:
                # change the value to False, to exit the main loop
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_0:
					pass

                if event.key == pygame.K_1:
                    pass

                if event.key == pygame.K_q:
                    print("Quitting")
                    pygame.quit()
                if event.key == pygame.K_DOWN:
                    pass
                if event.key == pygame.K_RIGHT:
					pass

                if event.key == pygame.K_UP:
					pass

                if event.key == pygame.K_LEFT:
					pass

                if event.key == pygame.K_a:
					pass

                if event.key == pygame.K_z:
					pass

                if event.key == pygame.K_SPACE:
					pass



		rate.sleep()


if __name__ == '__main__':
	try:
		teleop()
	except rospy.ROSInterruptException:
		pass


