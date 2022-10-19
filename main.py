import matplotlib.pyplot as plt

# step = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]
# measurement = [36.0, 43.0, 49.0, 54.0, 60.0, 66.0, 74.0, 82.0, 89.0, 94.0]
# predicted = [36.0, 40.30769230769231, 45.076388888888886, 49.84680025856496, 55.24347404760462, 60.953599518094194, 67.87737897567166, 75.37185280742302, 82.60382556312233, 88.6513517767754]
#
# step = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0, 37.0, 38.0, 39.0, 40.0, 41.0, 42.0, 43.0, 44.0, 45.0, 46.0, 47.0, 48.0, 49.0, 50.0, 51.0, 52.0, 53.0, 54.0, 55.0, 56.0, 57.0, 58.0, 59.0, 60.0, 61.0, 62.0, 63.0, 64.0, 65.0, 66.0, 67.0, 68.0, 69.0, 70.0, 71.0, 72.0, 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0, 81.0, 82.0, 83.0, 84.0, 85.0, 86.0, 87.0, 88.0, 89.0, 90.0, 91.0, 92.0, 93.0, 94.0, 95.0, 96.0, 97.0, 98.0, 99.0]
# measurement = [31.0, 33.0, 41.0, 43.0, 44.0, 48.0, 50.0, 58.0, 65.0, 68.0, 73.0, 77.0, 78.0, 82.0, 87.0, 93.0, 95.0, 99.0, 100.0, 107.0, 110.0, 115.0, 122.0, 127.0, 134.0, 139.0, 145.0, 149.0, 155.0, 161.0, 164.0, 166.0, 175.0, 176.0, 180.0, 185.0, 192.0, 193.0, 197.0, 198.0, 205.0, 206.0, 207.0, 209.0, 218.0, 226.0, 233.0, 241.0, 250.0, 258.0, 259.0, 267.0, 271.0, 275.0, 279.0, 280.0, 283.0, 287.0, 296.0, 301.0, 310.0, 319.0, 322.0, 326.0, 330.0, 339.0, 344.0, 345.0, 352.0, 353.0, 362.0, 365.0, 372.0, 374.0, 378.0, 386.0, 392.0, 398.0, 402.0, 406.0, 408.0, 417.0, 426.0, 432.0, 439.0, 442.0, 451.0, 454.0, 462.0, 468.0, 470.0, 475.0, 482.0, 489.0, 491.0, 494.0, 500.0, 505.0, 514.0, 520.0]
# predicted = [31.0, 32.1578947368421, 36.474747474747474, 39.497854077253216, 41.55091006835112, 44.47786491511188, 46.98055142879074, 51.97254750308763, 57.873450607658626, 62.4601881873688, 67.23405505980213, 71.65739415408314, 74.5301799278528, 77.91351901369653, 82.02909919626461, 86.99819787840698, 90.62248951699713, 94.41695239670557, 96.94570684156739, 101.49964231110073, 105.34974691524022, 109.72067872679601, 115.28240602095437, 120.58970768731139, 126.66369069651157, 132.25122983126383, 138.0255866899743, 142.99627624891482, 148.4331759384332, 154.1251231420989, 158.59779479266908, 161.95050833623515, 167.86107231174165, 171.54747282702374, 175.37591334392965, 179.73499347917155, 185.2902371292101, 188.78225412557498, 192.50435411052032, 194.9935213097833, 199.5257999603223, 202.45818801885468, 204.51533100733002, 206.54659196929907, 211.73423472560756, 218.1956908869987, 224.90107209112168, 232.19283071239505, 240.25831065666887, 248.2941324801733, 253.14318840661517, 259.41941536851755, 264.66466078510393, 269.34589168759413, 273.71856966442203, 276.5636456882807, 279.47889211386206, 282.88546077363156, 288.8254870293652, 294.33974303547313, 301.4328124857358, 309.3895964240016, 315.10128228988543, 320.0376867059662, 324.5499613282742, 331.09488125453726, 336.9400537656142, 340.59068086166894, 345.7583542409568, 349.03834488347593, 354.9091246631508, 359.47962946012467, 365.15053626475265, 369.1587630144078, 373.16326358595575, 378.977463385217, 384.8758184911694, 390.8202120685452, 395.88392286311534, 400.46584242284365, 403.87832173688327, 409.82158150810636, 417.14934415072156, 423.8757173870922, 430.7260256037298, 435.8323966903835, 442.7023263417833, 447.81943163996925, 454.24229918612343, 460.4736354643088, 464.78845388735493, 469.4136146325215, 475.1144218249909, 481.4036781619393, 485.7501826553598, 489.48680891315774, 494.2485950506247, 499.1182764464383, 505.8587213155185, 512.263793215153]

# step = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0, 37.0, 38.0, 39.0, 40.0, 41.0, 42.0, 43.0, 44.0, 45.0, 46.0, 47.0, 48.0, 49.0, 50.0, 51.0, 52.0, 53.0, 54.0, 55.0, 56.0, 57.0, 58.0, 59.0, 60.0, 61.0, 62.0, 63.0, 64.0, 65.0, 66.0, 67.0, 68.0, 69.0, 70.0, 71.0, 72.0, 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0, 81.0, 82.0, 83.0, 84.0, 85.0, 86.0, 87.0, 88.0, 89.0, 90.0, 91.0, 92.0, 93.0, 94.0, 95.0, 96.0, 97.0, 98.0, 99.0]
# measurement = [31.0, 40.0, 43.0, 47.0, 51.0, 59.0, 68.0, 69.0, 77.0, 83.0, 84.0, 85.0, 86.0, 92.0, 99.0, 102.0, 109.0, 115.0, 121.0, 130.0, 137.0, 143.0, 152.0, 153.0, 158.0, 164.0, 171.0, 177.0, 178.0, 181.0, 190.0, 196.0, 202.0, 203.0, 210.0, 213.0, 219.0, 227.0, 229.0, 237.0, 242.0, 243.0, 247.0, 253.0, 254.0, 259.0, 267.0, 274.0, 275.0, 281.0, 290.0, 298.0, 307.0, 310.0, 315.0, 323.0, 330.0, 332.0, 338.0, 339.0, 342.0, 348.0, 356.0, 362.0, 363.0, 371.0, 375.0, 377.0, 386.0, 394.0, 400.0, 402.0, 410.0, 413.0, 421.0, 427.0, 433.0, 434.0, 440.0, 442.0, 443.0, 444.0, 453.0, 458.0, 466.0, 471.0, 472.0, 473.0, 479.0, 484.0, 493.0, 494.0, 502.0, 508.0, 517.0, 521.0, 526.0, 527.0, 532.0, 540.0]
# predicted = [34.44444444444444, 39.029718621561805, 41.88251228711985, 45.25826297888224, 48.97851932817258, 55.13813279487772, 62.94650691731735, 66.96439460575517, 73.26223494909968, 79.43817987467882, 82.73460619574126, 84.7575714619875, 86.21735038150095, 90.25368455533939, 95.99462235602886, 100.22892033589561, 106.05740742701053, 112.02579823592991, 118.0560613176886, 125.80906159328897, 133.19369109292435, 139.85028425371348, 147.88027835137703, 151.9966595871464, 156.64274862448747, 162.0882251314271, 168.4523546589582, 174.65763277493224, 177.96703527717648, 181.12598931675987, 187.6091561729887, 193.86707842301902, 200.0253864816533, 203.31401652518835, 208.7242829611961, 212.81234187714907, 218.01103014682576, 224.83114258330383, 228.97757966906835, 235.33233512053357, 240.96834281253842, 244.02598606546505, 247.6387245891159, 252.6272029126707, 255.398476979973, 259.44969377957534, 265.76233815912354, 272.5099717709141, 276.05923042366413, 281.0196348407382, 288.2994912413803, 296.03999810605006, 304.549351905051, 310.00797896441253, 315.2476748127154, 322.08592280793073, 329.0660040687601, 333.2831871997031, 338.5389805183439, 341.4284743625055, 344.40172423744286, 349.1073893752631, 355.70946284757554, 362.0199714241453, 365.37591198747515, 371.3810711790667, 376.2973455423208, 379.6018122323721, 386.14933190600635, 393.5659636194718, 400.23670972584034, 404.31708970841044, 410.6426314751098, 415.13546994831995, 421.6434210509097, 427.91230410795896, 434.075459572028, 437.3662333755443, 442.2123231385225, 445.48575076254275, 447.49854545259944, 448.9538265467331, 454.68354582555475, 460.0431319242438, 466.9344012572397, 472.807681651945, 475.97025850935955, 477.93402956387877, 482.19325374968446, 486.90251493220865, 494.07130350016655, 497.8068174915864, 503.9798426345663, 510.1006050364436, 517.89362854167, 523.6005840268135, 528.9501028615684, 531.8810467171573, 536.0028773135515, 542.3467505708408]

# step = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0, 37.0, 38.0, 39.0, 40.0, 41.0, 42.0, 43.0, 44.0, 45.0, 46.0, 47.0, 48.0, 49.0, 50.0, 51.0, 52.0, 53.0, 54.0, 55.0, 56.0, 57.0, 58.0, 59.0, 60.0, 61.0, 62.0, 63.0, 64.0, 65.0, 66.0, 67.0, 68.0, 69.0, 70.0, 71.0, 72.0, 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0, 81.0, 82.0, 83.0, 84.0, 85.0, 86.0, 87.0, 88.0, 89.0, 90.0, 91.0, 92.0, 93.0, 94.0, 95.0, 96.0, 97.0, 98.0, 99.0]
# measurement = [36.0, 43.0, 47.0, 54.0, 58.0, 60.0, 64.0, 65.0, 71.0, 73.0, 80.0, 85.0, 87.0, 90.0, 94.0, 100.0, 101.0, 110.0, 111.0, 112.0, 121.0, 125.0, 129.0, 133.0, 138.0, 139.0, 148.0, 151.0, 158.0, 163.0, 165.0, 167.0, 168.0, 173.0, 178.0, 180.0, 186.0, 193.0, 200.0, 203.0, 208.0, 216.0, 223.0, 230.0, 233.0, 241.0, 244.0, 253.0, 260.0, 265.0, 271.0, 275.0, 279.0, 288.0, 295.0, 297.0, 306.0, 311.0, 317.0, 321.0, 322.0, 328.0, 332.0, 334.0, 337.0, 339.0, 346.0, 350.0, 358.0, 365.0, 372.0, 374.0, 383.0, 387.0, 395.0, 399.0, 404.0, 405.0, 406.0, 407.0, 413.0, 414.0, 417.0, 425.0, 428.0, 433.0, 434.0, 437.0, 443.0, 448.0, 456.0, 463.0, 469.0, 476.0, 485.0, 493.0, 495.0, 500.0, 504.0, 507.0]
# predicted = [37.89473684210526, 41.88102338722335, 45.41177699872124, 50.91809483072163, 55.55836921426825, 58.6814193935334, 62.33973664867516, 64.46702553797255, 68.87754937365584, 71.90762103842309, 77.28429034448729, 82.47854541624648, 85.8389098229865, 89.01208762135508, 92.69205108349217, 97.75693361855218, 100.47710452502794, 106.89451638665031, 110.184685207177, 112.15693657108504, 118.25915188151741, 123.17350282796268, 127.58725079573877, 131.7900291024575, 136.48957015186565, 139.05577479631805, 145.40830048767037, 149.8424680940443, 155.81086762477204, 161.2544963585465, 164.71995460778822, 167.35175030523098, 169.04654276348998, 172.6891405925586, 177.15260417032454, 180.20499034537872, 185.00539229777786, 191.1281343696824, 197.8081516850468, 202.38033444486751, 207.2355540171466, 213.96706985636882, 220.90364336661054, 227.92663455552287, 232.64335717015288, 239.31650608497054, 243.88579428723347, 251.08248167411597, 258.2150929386761, 264.14935669731307, 270.1642714730947, 275.04183125969837, 279.4400742887257, 286.564677921095, 293.66691085011695, 297.83135648019663, 304.8574306189076, 310.7467963200257, 316.74278965829996, 321.6123753627912, 324.2502421122971, 328.8759526896227, 333.16805863081214, 336.14822936958535, 339.16118173828323, 341.6022773873592, 346.73073442231544, 351.2347133532463, 357.81820546465974, 364.6923971733243, 371.6890987313402, 375.80907016026504, 382.8164014877199, 388.1121964675915, 395.02938448291036, 400.2871902681143, 405.43135270218517, 408.184934623278, 409.9310515830276, 411.2525914817562, 415.32356099404177, 417.6248660478254, 420.3517233080873, 426.1862810893148, 430.40216091256747, 435.1072233484241, 437.6757548768089, 440.51522968265965, 445.225904163769, 450.13948845368753, 456.8956009997269, 463.84254032703615, 470.28422807578687, 477.09865901673305, 485.2415193646006, 493.358549253196, 497.95066187633347, 502.81428051271723, 507.2066483075531, 510.8147444268051]

# step = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0, 37.0, 38.0, 39.0]
# measurement = [34.0, 39.0, 48.0, 53.0, 60.0, 61.0, 67.0, 76.0, 81.0, 87.0, 88.0, 93.0, 97.0, 98.0, 101.0, 103.0, 110.0, 117.0, 122.0, 123.0, 127.0, 136.0, 141.0, 149.0, 152.0, 158.0, 164.0, 165.0, 168.0, 173.0, 180.0, 188.0, 196.0, 205.0, 209.0, 215.0, 219.0, 228.0, 233.0, 236.0]
# predicted = [35.78947368421053, 38.56723144979551, 44.838516742037505, 50.286403399833816, 56.69955552734058, 59.90540456650723, 64.82135744329423, 72.23930469382034, 78.26585742379443, 84.32891322326174, 87.39601276410723, 91.65374334107261, 95.7933662136137, 98.07849302184684, 100.81399247465352, 103.13043152939518, 108.28730005505517, 114.59894749307557, 120.17575171077397, 123.04516576606032, 126.62036743809946, 133.49330645661217, 139.298304223023, 146.47560889451742, 151.20002723811422, 156.73370193944817, 162.59638047142553, 165.5820169617674, 168.6023091960044, 172.84101027778112, 178.77937599447765, 186.010901514668, 193.76816220507465, 202.34132071999898, 208.235384342631, 214.244579283772, 219.09626142945942, 226.48815462306948, 232.50413363158367, 236.75641408204933]

step = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0, 37.0, 38.0, 39.0]
measurement = [39.0, 48.0, 53.0, 59.0, 64.0, 68.0, 76.0, 83.0, 85.0, 93.0, 97.0, 106.0, 112.0, 116.0, 119.0, 124.0, 133.0, 140.0, 143.0, 145.0, 147.0, 151.0, 160.0, 165.0, 174.0, 177.0, 185.0, 193.0, 200.0, 209.0, 217.0, 223.0, 229.0, 236.0, 237.0, 241.0, 245.0, 254.0, 258.0, 259.0]
predicted = [40.206185567010316, 45.85668297956224, 50.65288215035504, 56.13375203482135, 61.34145914932307, 65.84585281397206, 72.49252326991896, 79.39150169111372, 83.36413844126452, 89.79823765867287, 94.79575284885117, 102.2455960211753, 108.86067739078852, 113.93064079942582, 117.77660735979892, 122.34356218390744, 129.6210502777841, 136.7726168595813, 141.45185788299696, 144.53593697485672, 146.9814660688964, 150.38234025115395, 157.19304138342164, 162.9467918657075, 170.69935922894066, 175.61918338067971, 182.432446880849, 190.00366207726606, 197.2728088476185, 205.63199568772058, 213.82205046764085, 220.73344165093732, 227.13297844460428, 233.9330974462259, 237.26069320727507, 241.01466236826417, 244.93931058585332, 251.95968065674282, 257.1918828302255, 259.89183492238783]



plot1 = plt.figure(1)
plt.scatter(step, measurement, color="RED")
plt.plot(step, predicted, color="TEAL")
plt.grid(True)

plt.show()