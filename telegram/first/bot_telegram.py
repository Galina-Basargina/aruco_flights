# Information: https://clover.coex.tech/en/snippets.html#navigate_wait

import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
from bot_telegram import *
import telebot
from time import sleep

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)


token = "6378043927:AAFoIgd_ipoo31xTcJnDEq3rXMT5JeOLsw8"
bot = telebot.TeleBot(token, parse_mode=None)

address = ""
item = ""
price = 0
flag = False

@bot.message_handler(commands=['start', 'help'])
def send_welcome(message):
    bot.reply_to(message, "Здравствуй!\nСколько будет весить твой груз? 1, 2 или 3 кг?\nТы можешь заказать доставку на ул. Ватутина, ул.Садовая, Витебский пр. и Невский пр.\nБудь внимателен - напиши вточности так же как написано в списке!\n")

@bot.message_handler(func=lambda m: True)
def echo_all(message):
    def navigate_wait(x=0, y=0, z=0, yaw=math.nan, speed=0.5, frame_id='body', tolerance=0.2, auto_arm=False):
        res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
        if not res.success:
            return res
        while not rospy.is_shutdown():
            telem = get_telemetry(frame_id='navigate_target')
            if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
                return res
        rospy.sleep(0.2)
    
    global address
    global item
    global price
    global flag

    if message.text == "1":
        bot.reply_to(message, "Ты заказал доставку на 1 кг")
        price += 300
        flag = True
    elif message.text == "2":
        bot.reply_to(message, "Ты заказал доставку на 2 кг")
        price += 400
        flag = True
    elif message.text == "3":
        bot.reply_to(message, "Ты заказал доставку на 3 кг")
        price += 500
        flag = True
    elif flag:
        if message.text == "ул. Ватутина":
            bot.reply_to(message, "Ты выбрал адрес ул. Ватутина")
            price += 600
            answer = f"Оплата полета составляет {price} рублей"
            print("Up")
            navigate_wait(z=0.5, frame_id='body', auto_arm=True)
            navigate_wait(z=0.5, frame_id='aruco_2')
            print("Down")
            land()
            rospy.sleep(8)
            bot.reply_to(message, "Дрон совершил посадку и доставил груз :)")

            print("Up")
            navigate_wait(z=0.5, frame_id='body', auto_arm=True)
            navigate_wait(z=0.5, frame_id='aruco_0')
            print("Down")
            land()
            rospy.sleep(8)

            bot.reply_to(message, answer)
            price = 0
            flag = False
            bot.reply_to(message, "Дрон вернулся на позицию и готов к новому заказу")
        elif message.text == "Витебский пр.":
            bot.reply_to(message, "Ты выбрал адрес Витебский пр.")
            price += 800
            answer = f"Оплата полета составляет {price} рублей"
            print("Up")
            navigate_wait(z=0.5, frame_id='body', auto_arm=True)
            navigate_wait(z=0.5, frame_id='aruco_8')
            print("Down")
            land()
            rospy.sleep(8)
            bot.reply_to(message, "Дрон совершил посадку и доставил груз :)")

            print("Up")
            navigate_wait(z=0.5, frame_id='body', auto_arm=True)
            navigate_wait(z=0.5, frame_id='aruco_0')
            print("Down")
            land()
            rospy.sleep(8)

            bot.reply_to(message, answer)
            price = 0
            flag = False
            bot.reply_to(message, "Дрон вернулся на позицию и готов к новому заказу")
        elif message.text == "ул. Садовая":
            bot.reply_to(message, "Ты выбрал адрес ул. Садовая")
            price += 600
            answer = f"Оплата полета составляет {price} рублей"
            print("Up")
            navigate_wait(z=0.5, frame_id='body', auto_arm=True)
            navigate_wait(z=0.5, frame_id='aruco_17')
            print("Down")
            land()
            rospy.sleep(8)
            bot.reply_to(message, "Дрон совершил посадку и доставил груз :)")

            print("Up")
            navigate_wait(z=0.5, frame_id='body', auto_arm=True)
            navigate_wait(z=0.5, frame_id='aruco_0')
            print("Down")
            land()
            rospy.sleep(8)
            
            bot.reply_to(message, answer)
            price = 0
            flag = False
            bot.reply_to(message, "Дрон вернулся на позицию и готов к новому заказу")
        elif message.text == "Невский пр.":
            bot.reply_to(message, "Ты выбрал адрес Невский пр.")
            price += 800
            answer = f"Оплата полета составляет {price} рублей"
            print("Up")
            navigate_wait(z=0.5, frame_id='body', auto_arm=True)
            navigate_wait(z=0.5, frame_id='aruco_14')
            print("Down")
            land()
            rospy.sleep(8)
            bot.reply_to(message, "Дрон совершил посадку и доставил груз :)")

            print("Up")
            navigate_wait(z=0.5, frame_id='body', auto_arm=True)
            navigate_wait(z=0.5, frame_id='aruco_0')
            print("Down")
            land()
            rospy.sleep(8)

            bot.reply_to(message, answer)
            price = 0
            flag = False
            bot.reply_to(message, "Дрон вернулся на позицию и готов к новому заказу")
        else:
            bot.reply_to(message, "Прости, но я не понял что ты написал. Проверь свое сообщение")
    else:
        bot.reply_to(message, "Ты не выбрал вес груза, напиши сначала его")

    
bot.infinity_polling()
