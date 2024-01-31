# print(sum(x for x in [13, 18, 8, 21, 20]))

# a_list = [1, 4, 9, 16, 25, 36, 49, 64, 81, 100]
# result = (a_list[0], a_list[-1])
# print(result)

# result = []
# for x in range(15, 25):
#     result.append(x)
# print(result)

# print(4 * 3.14 * 7**2)

# a = ['Нулевой элемент', 2, 5, 8, (3.5,55)]
# print('a[0] = ', a[0])
# print('a[-1 = ', a[-1])

# first = 100
# second = 12.123445643
# print(f'{first} - {second} равно ', first - second)

# a = 123
# b = '123'
# print(a)
# print(b)
#
# print(type(a))
# print(type(b))

#

#

# a = int(input())
# if a % 2 == 1:
#     print('Пойду позову еще одного друга')
# elif (a >= 5) and (a <= 10):
#     print('Начинаем игру')
# elif a > 10:
#     print('Придётся разделиться на 2 группы')
# elif a < 5:
#     print('Зовём еще друзей!')

# strike = 1
# head = 9
# sum_head = head
# while strike != 6:
#     head *= 2
#     sum_head += head
#     strike += 1
# print(sum_head)

page_nums = [52, 12, 64, 324, 75, 32, 64, 12, 7, 31, 56, 12, 34, 1, 3, 73, 2]
# result = 0
# for x in page_nums:
#     result += x
# print(result)

# max_page = page_nums[0]
# for x in page_nums:
#     if x > max_page:
#         max_page = x
# print(max_page)

# book_d = {"Приключение Тома Сойера": 133, "Малыш и Карлсон": 62, "Война и мир": 5, "Сказка о рыбаке и рыбке": 15}
# print(book_d)
# book_d.update({"Колобок": 10})
# print(book_d)

# str = 'My name is Michele'.split(' ')
# print(' '.join(str[::-1]))

# spisok_1 = [1, 2, 4, 2, 4, 5, 2, 3, 2]
# spisok_2 = [4, 2, 3, 2, 1, 4, 2, 5, 2, 5]
# spisok_3 = [2, 4, 1, 2, 4, 5, 4, 5]
# print(sum(spisok_1 + spisok_2 + spisok_3))

# print(sum([x for x in range(0, 30, 2)]))

# def el(x: str) -> int:
#     if x == 'J':
#         return 11
#     elif x == 'Q':
#         return 12
#     elif x == "K":
#         return 13
#     elif x == 'A':
#         return 14
#     else:
#         return int(x)
#
# def winner(l):
#     win = None
#     for p1, p2, p3 in l:
#         player = el(p1) + el(p2) + el(p3)
#         if win is None:
#             win = player
#         elif player < win:
#             win = player
#     return win
#
# print(winner(
#     [['A', 'J', '2'],
#      ['10', '10', '7'],
#      ['J', 'J', '3'],
#      ['4', 'A', 'A'],
#      ['6', '7', 'A']
#     ]
# ))

# num = [386, 462, 47, 418, 907, 344, 375, 823,
#        566, 328, 626, 949, 687, 248, 6, 7, 14]
# print(len([x for x in num if x % 2 == 0]))

# result = 0
# while True:
#     x = int(input())
#     result += x
#     if x == 0:
#         print(result)
#         break

# data = [
#     [15, 3, 0.1],
#     [14, 2.5, 0.08],
#     [16, 3.5, 0.09],
#     [20, 5, 0.12],
#     [17, 4, 0.11]
# ]
# print(round(min([(s+n)*(1.0+p) for s,n,p in data]), 1))

# name = ['Иванов', 'Петров', 'Андреев', 'Сидоров', 'Яшин', 'Смирнов']
# print(sorted(name))

# a = (2, 4)
# b = (7, 11)
# print(((a[0]-b[0])**2 + (a[1]-b[1])**2) ** 0.5)

# print([x for x in range(2, 11, 2)])

# numb = 7
# seasons = ['Зима', 'Весна', 'Лето', 'Осень']
# for s in seasons:
#     numb += 1
#     if s == "Лето":
#         print(numb)

# age = 18
# year = 2024
# print(year - age)

# month = ['декабрь', 'январь', 'февраль', 'март', 'апрель', 'май', 'июнь', 'июль', 'август', 'сентябрь', 'октябрь', 'ноябрь']
# season = ['Зима', 'Весна', 'Лето', 'Осень']
# print(month)
# print(season)
# year = {}
# for i, m in enumerate(month):
#     s = season[i // 3]
#     # print(i, m, s)
#     if s not in year:
#         year.update({s: [m]})
#     else:
#         year[s].append(m)
# print(year)

# str = ''
# for i in range(6):
#     if i == 0:
#         str = '0'
#     else:
#         str = f'{i} {i-1} {str}'
#     print(str)

# for x in range(1, 11):
#     print(f'{x} * 120 = {x*120}')

# salary = int(input())
# rent = int(input())
# money = salary - rent
# if money < 0:
#     print('Ошибка! Или меняйте работу, или меняйте место жительства!')
# elif money >= 15000 and money <= 30000:
#     print('Можно жить спокойно')
# elif money > 30000:
#     print('Можно брать ипотеку!')
# else:
#     print('В этом месяце живём на макаронах....')

