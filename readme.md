# Daljinsko upravljanje robotskom rukom preko lokalne mreže

Predmeti: Operativni sistemi u realnom vremenu i Osnovi računarskih mreža\
Autori: Simon Radosavljević, Vuk Todorović, Milan Žeželj i Anja Ekres

## Uvod

<div style="text-align: justify"> 
Ovaj projekat predstavlja realizaciju daljinskog kontrolisanja robotizovane ruke preko lokalne mreže. Urađen je kao završni projekat iz predmeta Operativni sistemi u realnom vremenu i predmeta Osnovi računarskih mreža. Ova dokumentacija sadrži opis toka programa, opis toka rada zasebnih delova sistema i detalje vezane za mrežnu komunikaciju. Sistem iz tri dela. Prvi deo predstavlja klijent koji se pokreće na računaru koji se koristi za daljinsko upravljanje rukom. Drugi deo je server koji se pokreće na Raspberry Pi računaru koji je neposredno povezan sa robotskom rukom. Treći deo sistema predstavlja drajver robotske ruke koji komunicira sa serverom na Raspberry Pi-u. Svi delovi su detaljno opisani u nastavku teksta. 

## Tok programa i daljinskog upravljanja

Na početku rada potrebno je kompajlirati i pokrenuti server i propratne programe na Raspberry Pi-u i kompajlirati i pokrenuti server na računaru koji je namenjen da se koristi kao daljinski kontroler (u nastavku teksta PC).

Pri startovanju PC klijenta, pokreće se procedura pretraživanja aktivnih servera korsteći UDP broadcast protokol. Zatim se na ekranu ispisuje numerisana lista dostupnih servera zajedno sa svojim korisničkim imenima koja su podešena pri pokretanju servera na Raspberry Pi-u. Nakon toga, korisnik može sa liste da izabere server i pošalje komandu na server. PC klijent će pokušati da uspostavi TCP konekciju sa serverom i tako obezbedi slanje i primanje komande. Nakon izvršavanje komande nad robotskom rukom, server će klijentu vratiti povratnu informaciju iz drajvera. Potom se prethodno uspostavljena TCP konekcija raskida i korisnik ponovo dobija opciju da bira server na koji će poslati komandu. Program se terminira slanjem komande „exit”.

## Detaljniji opis delova sistema

### PC klijent

PC klijent pretražuje lokalnu mrežu za aktivnim serverima slanjem unapred dogovorene poruke preko broadcast adrese koristeći UDP protokol. Zatim počinje da prima poruke na istom portu. Za svaku primljenu poruku, PC klijent čuva poslato korisničko ime i IPV4 adresu sa koje je poruka primljena. Kada primi poruku, PC klijent šalje potvrdu o prijemu poruke. Nakon izvršenog pretraživanja, PC klijent na konzoli ispisuje listu svih dostupnih servera. Zatim na PC-u se unosi komanda formata “index w/r servo_idx w ? duty : null” i šalje TCP zahtev na željeni server. Nakon slanja, klijent ispisuje povratne informacije drajvera na ekran. Potom se raskida TCP konekcija i čeka da se unos nove komande nakon koje će se ponovo inicirati TCP konekcija.

### Raspberry Pi server

Raspberry Pi server čeka UDP signal (poslat preko broadcast-a) i šalje svoje podatke nazad klijentu preko UDP-a. Ponavlja slanje podataka dok ne dobije potvrdu o prijemu. Nakon potvrde o slanju očekuje TCP koneckiju. Po uspostavljanju TCP konekcije, očekuje komandu koje zatim prosledjuje drajveru. Server šalje povratnu informaciju sa drajvera klijentu i TCP konekcija se raskida. Nakon toga ponovo očekuje novu TCP konekciju.

### Drajver / ROS (Raspberry Pi)

Implementacija drajvera se nalazi u 'SW/Driver/main.c' fajlu. Kada se drajver kompajlija on napravi fajl drajvera u ‘/dev/servo_ctrl’ i na akcije pisanja i čitanja u taj fajl on izvrsava komande nad servo uređajima. Test aplikacija radi tako što prima string kao argument komandne linije u formatu ./test_app_servo_ctrl w servo_idx duty (za  pisanje) ili ./test_app_servo_ctrl r servo_idx (za čitanje). U folderu test aplikacije je fajl ‘test_app_servo_ctrl.c’ koji upravo za to i služi, da cita i pise fajl drajvera ono sto drajver ocekuje da bi upravljao servo motorima:
```
./test_app_servo_ctrl w servo_idx duty //Za pisanje
./test_app_servo_ctrl r servo_idx //Za citanje
```
- w - oznaka da se radi o akciji pisanja u fajl drajvera odnosno rotiranje servo motora
- r - oznaka da se radi o akciji čitanja stanja servo uredjaja (vraća na koji ugao je rotiran) 
- servo_idx - oznacava id servo uređaja sa kojim se radi posto ih ima ukupno 4 pa ce dakle ovo biti vrednosti 0, 1, 2 ili 3 
- duty - posto se servo motorom upravlja preko PWM pina, vrednost koja govori o rotaciji servo motora je faktor ispune PWM signala odnosno duty u našem kodu. Očekuje vrednost od 0 do 1000 sto će se mapirati na određene uglove. Test aplikacija sa arugmenata uzima taj string i parsira ga u promenljive funckijom parse_args.
- DEV_FN označava lokaciju fajla drajvera.
- N_SERVOS označava da postoje 4 servo motora.

## Detalji mrežne komunikacije

### Port

UDP Broadcast: 25565\
TCP: 25566

### UDP broadcast

PC šalje poruku: “ping”\
Kada malina dobije “ping” poruku, odgovori sa detaljima: “$username”\
PC prima ime maline, šalje “Received details”, i ispisuje ime radi selektovanja željenog servera.\
Malina čeka “Received details” poruku od PC klijenta. Nakon što primi potvrdu, otvara port za TCP konekciju.

### TCP connection

Korisnik bira malinu iz liste i PC klijent inicira TCP konekciju na izabrani server. \
PC šalje read/write poruku u formatu “w/r servo_idx w ? duty : null”\
Server prima poruku i šalje nazad povratnu informaciju od drajvera, ili “Error: errorMsg”.

## Development notes

- Koristicemo kod iz vežbe 5 gde je uradjen kompletan drajver za upravljanje servo motorima u zglobovima robotske ruke.
- Projekat se sastoji od 2 foldera ROS i SW, gde mi za potrebe ovog projekta ne koristimo ROS.
- U SW folderu imamo folder drajvera 'Driver/servo_ctrl' i folder aplikacije za testiranje drajvera iz korisničkog prostora 'Test/test_app_servo_ctrl'.
- U folderu drajvera nam je od najveceg znacaja main.c gde postoji implementiran drajver.
- Kada se drajver kompajlira on napravi fajl drajvera '/dev/servo_ctrl' i na akcije pisanja i citanja u taj fajl on izvršava komande nad servo uređajima.
- U folderu test aplikacije nalazi se kod programa 'test_app_servo_ctrl.c' koji služi za čitanje i pisanje u fajl drajvera ono što drajver očekuje da bi upravljao servo motorima.
- Test aplikacija radi tako što prima string kao argument komandne linije u formatu.
```
./test_app_servo_ctrl w servo_idx duty //Za pisanje
./test_app_servo_ctrl r servo_idx //Za citanje
```
Gde je:
- **w** - oznaka da se radi o akciji pisanja u fajl drajvera odnosno rotiranje servo motora
- **r** - oznaka da se radi o akciji citanja stanja servo uredjaja (vraca na koji ugao je rotiran)
- **servo_idx** - oznacava id servo uredjaja sa kojim se radi posto ih ima ukupno 4 pa ce dakle ovo biti vrednosti 0, 1, 2 ili 3
- **duty** - posto se servo motorom upravlja preko PWM pina, vrednost koja govori o rotaciji servo motora je faktor ispune PWM signala odnosno duty u nasem kodu (ovo     logicno nije potrebno kod operacije citanja jer tada iz drajvera treba procitati taj duty). Ocekuje vrednost od 0 do 1000 sto ce se mapirati na odredjene uglove.

<br />


- Test aplikacija sa argumenta uzima taj string i parsira ga u promenljive funckijom parse_args
- Promenljive su op - tip operacije odnosno r ili w, servo_idx i duty koji vec znate sta su
- Na vrhu fajla imaju 2 konstante: DEV_FN koja predstavlja lokaciju fajla drajvera koji se koristi i N_SERVOS koja oznacava da ima 4 servo motora
- Kasnije se otvara fajl drajvera i u njega se upisuje komanda (ako je mod 'w') ili se iz njega cita stanje (ako je mod 'r')
- Cela poenta je da treba modifikovati ovaj fajl da ne cita i ne parsira argumente sa komandne linije nego da npr sa mreze dobije string u istom formatu u kom bi dobio i sa komandne linije i onda samo da pustite dalje da se to sve odradi sa tim argumentima
- Ja vam predlazem da obrisete proveru argumenata skroz i samo da ih direktno kada procitate string sa mreze upisete u promenljive
- Druga opcija vam je da u skroz drugi fajl napisete server i sve i onda kada taj server dobije komandu da onda on bkv pokrene komandu npr: '/test_app_servo_ctrl w 2 560'
- Svakako vam preporucujem da vecinu mrezne aplikacije napisete odvojeno u drugi fajl da bi normalno mogli da testirate i tek kad je zavrsite da dodate taj mrezni kod za razberi u ovaj kod testne aplikacije
- Ostaje nam samo kada sve to zavrsimo da skontamo kako da povezemo tu ruku kada dodjemo tamo da ne budemo idioti, mada testna aplikacija koliko ja kontam svakako u terminal ispisuje koja komanda je zadata tako da ono mozda nam priznaju samo to bez povezivanja, mada svakako treba znati da povezemo te servoe da budemo 100% sigurni
- Dodatni predlog je da workflow bude otprilike da kad se PC i malina povezu, samo na PC ukucate komandu koju bi ste inace kucali u testnoj aplikaciji u komandnoj liniji i samo je sibnete kroz mrezu bez ikakve provere i pustite testnoj aplikaciji da se cima oko toga jer ima vec uradjeno to
</div>