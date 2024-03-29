import json
import requests
import telebot

drone_service = '127.0.0.1'
token = ""
bot = telebot.TeleBot(token, parse_mode=None)
weight_price = None
delivery_price = None
street_marker = None

def run(command):
    url = f'http://{drone_service}:8081/'
    #command = {'do': 'Land'}
    #command = {'do': 'Move', 'street': 'Vatutina', 'house': 50}
    #command = {'do': 'Start', 'altitude': 4}
    res = requests.get(url, params=command)
    data = res.json()
    print(data)
    return data

@bot.message_handler(commands=['start', 'help'])
def send_welcome(message):
    global delivery_price
    global weight_price
    global street_marker
    weight_price = None
    delivery_price = None
    street_marker = None


    bot.reply_to(message,
"""Здравствуй!
Сколько будет весить твой груз? 1, 2 или 3 кг?
Ты можешь заказать доставку на ул. Ватутина, ул.Садовая, Витебский пр. и Невский пр.
Будь внимателен - напиши в точности так же как написано в списке!
Ты можешь отменить заказ написав 'Отмена заказа'"""
   )

@bot.message_handler(commands=['status'])
def send_status(message):
    response = run({'do': 'Status'})
    bot.reply_to(message, json.dumps(response).encode())

@bot.message_handler(func=lambda m: True)
def echo_all(message):
    global delivery_price
    global weight_price
    global street_marker
    
    text = ''
    if message.text == "1":
        text = "Ты заказал доставку на 1 кг\n"
        weight_price = 300
    elif message.text == "2":
        text = "Ты заказал доставку на 2 кг\n"
        weight_price = 400
    elif message.text == "3":
        text = "Ты заказал доставку на 3 кг\n"
        weight_price = 500

    elif message.text == "ул. Ватутина":
        text = "Ты выбрал адрес ул. Ватутина\n"
        street_marker = 1
        delivery_price = 600
    elif message.text == "Витебский пр.":
        text = "Ты выбрал адрес Витебский пр.\n"
        street_marker = 2
        delivery_price = 800
    elif message.text == "ул. Садовая":
        text = "Ты выбрал адрес ул. Садовая\n"
        street_marker = 3
        delivery_price = 600
    elif message.text == "Невский пр.":
        text = "Ты выбрал адрес Невский пр.\n"
        street_marker = 4
        delivery_price = 800
    
    elif message.text == "Отмена заказа":
            weight_price = None
            delivery_price = None
            street_marker = None
            bot.reply_to(message, "Вы отменили свой заказ")
            return

    else:
        bot.reply_to(message, "Прости, но я не понял что ты написал. Проверь свое сообщение")
        return

    if weight_price is None:
        bot.reply_to(message, text+"Ты не выбрал вес груза, для продолжения требуется его указать")
        return
    if delivery_price is None:
        bot.reply_to(message, text+"Ты не выбрал адрес доставки, для продолжения требуется его указать")
        return
    bot.reply_to(message, text+"Стоимость доставки: "+str(delivery_price+weight_price))

    response = run({'do': 'Move', 'street': street_marker})
    text = ''
    if response.get('error'):
        text += 'Ошибка: '+str(response.get('error'))
        street_marker = None
    else:
        if response.get('speed'):
            text += 'Дрон двигается со скоростью '+str(response.get('speed'))+"м/с.\n"
        if response.get('altitude'):
            text += 'Дрон находится на высоте '+str(response.get('altitude'))+"м.\n"
            print(message.text)
            street_marker = None
            weight_price = None
            delivery_price = None
        message.text = ""

    bot.reply_to(message, text)

if __name__ == "__main__":
    bot.infinity_polling()

