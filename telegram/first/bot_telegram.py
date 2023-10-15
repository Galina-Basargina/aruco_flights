import telebot
from time import sleep


def bot(result):
	token = "6378043927:AAFoIgd_ipoo31xTcJnDEq3rXMT5JeOLsw8"
	bot = telebot.TeleBot(token, parse_mode=None)

	address = ""
	item = ""
	@bot.message_handler(commands=['start', 'help'])
	def send_welcome(message):
		bot.reply_to(message, "Выберите адрес")
		bot.reply_to(message, "Улица Ватутина - 1")
		bot.reply_to(message, "Невский проспект - 2")
		bot.reply_to(message, "Выберите размер груза")
		bot.reply_to(message, "1 коробка 20х20 см - 3")
		bot.reply_to(message, "2 коробки 20х20 см - 4")

	@bot.message_handler(func=lambda m: True)
	def echo_all(message):
		global address
		global item
		if message.text == "1":
			bot.reply_to(message, "Vatutina")
			result = "Улица Ватутина"
			bot.polling()
		elif message.text == "2":
			bot.reply_to(message, "Nevskiy")
			result = "Невский проспект"
			bot.polling()
		elif message.text == "3":
			bot.reply_to(message, "1 korobka")
			result = "1 коробка 20х20 см"
			bot.polling()
		elif message.text == "4":
			bot.reply_to(message, "2 korobki")
			result = "2 коробки 20х20 см"
			bot.polling()
		elif message.text == "5":
			address = ""
			item = ""
			bot.polling()
	bot.infinity_polling()
