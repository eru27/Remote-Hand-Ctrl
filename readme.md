Projekat iz predmeta Operativni sistemi za rad u realnom vremenu i Osnovi računarskih mreža.

# Development notes
- Koristicemo kod iz vezbe 5 gde je uradjen kompletan drajver za upravljanje servo motorima u zglobovima robotske ruke
- Projekat se sastoji od 2 foldera ROS i SW, predlazem da ROS samo ignorisemo posto nigde nisu rekli da moramo da ga koristimo i povezujemo
- U SW folderu imamo folder drajvera 'Driver/servo_ctrl' i folder aplikacije za testiranje drajvera iz korisnickog prostora 'Test/test_app_servo_ctrl'
- U folderu drajvera nam je od najveceg znacaja main.c gde je drajver vec implementiran
- Kada se drajver kompajlira on napravi fajl drajvera u '/dev/servo_ctrl' i na akcije pisanja i citanja u taj fajl on izvrsava komande nad servo uredjajima
- U folderu test aplikacije nalazi se fajl 'test_app_servo_ctrl.c' koji upravo za to i sluzi, da cita i pise u fajl drajvera ono sto drajver ocekuje da bi upravljao servo motorima
- Test aplikacija radi tako sto prima string kao argument komandne linije u formatu
```
./test_app_servo_ctrl w servo_idx duty //Za pisanje
./test_app_servo_ctrl r servo_idx //Za citanje
```
gde je: <br />
  **w** - oznaka da se radi o akciji pisanja u fajl drajvera odnosno rotiranje servo motora <br />
  **r** - oznaka da se radi o akciji citanja stanja servo uredjaja (vraca na koji ugao je rotiran) <br />
  **servo_idx** - oznacava id servo uredjaja sa kojim se radi posto ih ima ukupno 4 pa ce dakle ovo biti vrednosti 0, 1, 2 ili 3 <br />
  **duty** - posto se servo motorom upravlja preko PWM pina, vrednost koja govori o rotaciji servo motora je faktor ispune PWM signala odnosno duty u nasem kodu (ovo     logicno nije potrebno kod operacije citanja jer tada iz drajvera treba procitati taj duty). Ocekuje vrednost od 0 do 1000 sto ce se mapirati na odredjene uglove<br />
- Test aplikacija sa argumenta uzima taj string i parsira ga u promenljive funckijom parse_args
- Promenljive su op - tip operacije odnosno r ili w, servo_idx i duty koji vec znate sta su
- Na vrhu fajla imaju 2 konstante: DEV_FN koja predstavlja lokaciju fajla drajvera koji se koristi i N_SERVOS koja oznacava da ima 4 servo motora
- Kasnije se otvara fajl drajvera i u njega se upisuje komanda (ako je mod 'w') ili se iz njega cita stanje (ako je mod 'r')
- Cela poenta je da treba modifikovati ovaj fajl da ne cita i ne parsira argumente sa komandne linije nego da npr sa mreze dobije string u istom formatu u kom bi dobio i sa komandne linije i onda samo da pustite dalje da se to sve odradi sa tim argumentima
- Ja vam predlazem da obrisete proveru argumenata skroz i samo da ih direktno kada procitate string sa mreze upisete u promenljive
- Druga opcija vam je da u skroz drugi fajl napisete server i sve i onda kada taj server dobije komandu da onda on bkv pokrene komandu npr: '/test_app_servo_ctrl w 2 560'
- Svakako vam preporucujem da vecinu mrezne aplikacije napisete odvojeno u drugi fajl da bi normalno mogli da testirate i tek kad je zavrsite da dodate taj mrezni kod za razberi u ovaj kod testne aplikacije
- Ostaje nam samo kada sve to zavrsimo da skontamo kako da povezemo tu ruku kada dodjemo tamo da ne budemo idioti, mada testna aplikacija koliko ja kontam svakako u terminal ispisuje koja komanda je zadata tako da ono mozda nam priznaju samo to bez povezivanja, mada svakako treba znati da povezemo te servoe da budemo 100% sigurni
