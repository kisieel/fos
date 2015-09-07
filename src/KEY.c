#include "stm32l1xx.h"
#include "KEY.h" 

#define PERIOD_1S      100
#define PERIOD_750MS   75
#define PERIOD_500MS   50
#define PERIOD_100MS   10
#define PERIOD_30MS    3

// ***********************************************************************
static unsigned int keys;
// Wewnetrzna zmienna w kt?rej przechowywany jest stan klawiszy
// Zawartosc tej zmiennej pobierana jest przez program za pomoca funkcji
//  GetKeys(); jesli zmienna keycnt jest wieksza od zera.
// ***********************************************************************

// ***********************************************************************
static unsigned int keycnt;
// Wewnetrzna zmienna w kt?rej przechowywana jest licznik autorepetycji
// Zwiekszany okresowo w momencie gdy sa nacisniete klawisze i ich stan
// nie zmienia sie przez dluzszy czas. Zmniejszana podczas kazdego
// wywolania funkcji GetKeys();
// ***********************************************************************

// ***********************************************************************
static unsigned int keytime;
// Wewnetrzna zmienna reprezentujaca czas od ostatniego nacisniecia
// klawiszy. Zwracana do programu za pomoca funkcji KeysTime();
// Jesli nie ma wcisnietego zadnego klawisza, zmienna utrzymuje wartosc 0;
// ***********************************************************************

// ***********************************************************************
static unsigned int prevKeybSt;
// Wewnetrzna zmienna pamietajaca stan klawiszy z poprzedniego wywolania
// funkcji KeybProc(). Sluzy do wykrycia zmiany stanu klawiatury
// ***********************************************************************

// ***********************************************************************
static unsigned int arTime;
// Wewnetrzna zmienna reprezentujaca czas w kt?rym ma nastapic zwiekszenie
// licznika klawiatury. Zwiekszana o odpowiedni czas zalezny o czas?w
// autorepetycji po kazdym zwiekszeniu licznika klawiatury
// ***********************************************************************

// ***********************************************************************
static unsigned char arIndex ;
// Wewnetrzna zmienna indeksujaca tablice z kolejnymi czasami autorepetycji
// ***********************************************************************

// ***********************************************************************
static unsigned char keyblock;
// Wewnetrzna zmienna ustawiana na 1 funkcja KeybLock() lub ClrKeyb()
// z parametrem KBD_LOCK. Jesli zmienna ma wartosc 1 obsluga klawiatury
// zostaje zablokowana do momentu zwolnienia wszystkich klawiszy.   
// ***********************************************************************





// ***********************************************************************
// Domyslna tablica z kolejnymi czasami autorepetycji. Ostatnia pozycja r?zna
// od zera jest czasem autorepetycji nieskonczonej. Ilosc pozycji dowolna,
// ostatnia pozycja musi byc r?wna 0;
static const unsigned short DefaultAutoRepeatTab[] =
{
	PERIOD_30MS,
	PERIOD_1S,
	PERIOD_500MS,
	PERIOD_500MS,
	PERIOD_100MS,
	PERIOD_100MS,
	PERIOD_100MS,
	PERIOD_100MS,
	PERIOD_100MS,
	PERIOD_30MS,
   0
};

// KEY_1       PA1
// KEY_2       PA2

static unsigned short 
    * volatile _art = (unsigned short *)DefaultAutoRepeatTab,
    * volatile art = (unsigned short *)DefaultAutoRepeatTab;

// ***********************************************************************
// Funkcja kofigurujaca timer do wywolywania przerwania oraz funkcja
// przerwania.
// ***********************************************************************

void KEY_init(void)
{
//	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	GPIO_config(0x0A, 1, GPIO_MODE_Input, GPIO_PULL_Floating, 0, 0, 0);
	GPIO_config(0x0A, 2, GPIO_MODE_Input, GPIO_PULL_Floating, 0, 0, 0);
	GPIO_config(0x0A, 3, GPIO_MODE_Input, GPIO_PULL_Floating, 0, 0, 0);
	
//	TIM6->ARR = 10000;
//	TIM6->PSC = 32;
//	
//	TIM6->DIER |= TIM_DIER_UIE;
//	NVIC_EnableIRQ(TIM6_IRQn);
//	
//	TIM6->CR1 |= TIM_CR1_CEN;
}

//void TIM6_IRQHandler()
//{
//	if (TIM6->SR & TIM_SR_UIF) {
//		KeybProc();
//		TIM6->SR &= ~(TIM_SR_UIF);
//	}
//}

// ***********************************************************************
// Funkcja dostarczajaca surowy stan klawiatury.
// Stan wysoki dla wcisnietych klawiszy. Jesli funkcja KeybProc() 
// bedzie wywolywana z przerwania to funkcja GetKeybSample() musi
// trwac jak najkr?cej
// ***********************************************************************
unsigned int
GetKeybSample( void )
{	
  return (((GPIOA->IDR & GPIO_IDR_IDR_1) >> 1) 
		    | ((GPIOA->IDR & GPIO_IDR_IDR_2) >> 1)
	      | ((GPIOA->IDR & GPIO_IDR_IDR_3) >> 1)) & ANYKEY;   
}

// ***********************************************************************
// Funkcja wywolywana z jakiegos przerwania w tym przypadku co 10ms
// ***********************************************************************
void
KeybProc( void )
{
   unsigned int realKeybSt;

   // Pobranie stanu klawiszy
   realKeybSt = GetKeybSample();
   
   // Sprawdzenie czy stan sie zmienil
   if( prevKeybSt != realKeybSt)
   {
      // Stan sie zmienil wiec resetowanie klawiatury i wyjscie z funkcji
      ClrKeyb(KBD_NOLOCK);      
      return;
   }

   // Sprawdzenie czy brak nacisnietych klawiszy lub klawiatura zablokowana
   if( !realKeybSt || keyblock)
   {
      // Ponowne sprawdzenie czy brak nacisnietych klawiszy
      // Jesli tak to odblokowanie klawiatury
      if( !realKeybSt ) keyblock = 0;
      return;
   }
      // Zwiekszenie licznika czasu klawiatury
    keytime++;
      // Zachowanie stanu klawiszy
    keys = realKeybSt;
      
      // Obsluga autorepetycji
      // Sprawdzenie czy licznik czsu klawiatury osiagnal czas nastepnej
      // autorepetycji
    if( keytime >= arTime)
    {
//			GPIOB->ODR |= GPIO_ODR_ODR0 | GPIO_ODR_ODR1;
         // Zwiekszenie licznika autorepetycji
      keycnt++;
         // Obliczenie kolejnego czasu autorepetycji

      _art = art;
      if( _art[ arIndex + 1 ]) arIndex++;
      arTime += _art[ arIndex ];      
    }
}



// ***********************************************************************
// Funkcja zwraca stan klawiszy do programu jesli licznik autorepetycji
// r?zny od zera
// ***********************************************************************
unsigned int
GetKeys( void )
{
   if( keycnt )
   {
      keycnt--;        // Jesli funkcja KeybProc() bedzie wywolywana
                       // z przerwania to ta instrukcja musi byc wykonana
                       // atomowo.
      return keys ;
   }
   return 0;
}


// ***********************************************************************
// Funkcja zwraca czas wciskania aktualnej kombinacji klawiszy
// ***********************************************************************
unsigned int
KeysTime( void )
{
    return keytime;
}

// ***********************************************************************
// Funkcja zwraca stan klawiszy zgodnie z podana maska jako argument funkcji
// ***********************************************************************
unsigned int
IsKeyPressed( unsigned int mask )
{
    return keys & mask;
}

// ***********************************************************************
// Funkcja zwraca stan klawiszy zgodnie z ustawiona maska podana jako
// argument funkcji, jest brany pod uwage licznik autorepetycji
// Ale pobranie stanu klawiwszy nie zminiejsza licznika autorepetycji
// ***********************************************************************
unsigned int
IsKey( unsigned int mask )
{
    if(keycnt)
    {
       return keys & mask ;
    }
    return 0;
}


// ***********************************************************************
// Funkcja resetuje stan klawiatury. Jako parametr nalezy podac stala
// KBD_LOCK lub KBD_NOLOCK, kt?re odpowiednio blokuja lub nie klawiature
// Jesli funkcja KeybProc() bedzie wywolywana z przerwania to funkcja
// ClrKeyb() musi byc wykonana atomowo.
// ***********************************************************************

void
ClrKeyb( int lock )
{
    prevKeybSt = GetKeybSample();
    keys = 0;
    keytime = 0;
    keycnt = 0;
    arIndex = 0;
    arTime = _art[0];
    if( lock ) keyblock = 1;
}


// ***********************************************************************
// Funkcja blokuje klawiature. Odblokowanie nastepuje po zwolnieniu
// wszystkich klawiszy.
// ***********************************************************************
void
KeybLock( void )
{
   keyblock = 1;
}


// ***********************************************************************
// Funkcja podmienia tablice z miedzyczasami autorepetycji. Nowa tablica
// powinna byc zgodna z wczesniej opisanym formatem
// ***********************************************************************
void
KeybSetAutoRepeatTimes( unsigned short * AutoRepeatTab )
{
    if( AutoRepeatTab == KBD_DEFAULT_ART )
        art = (unsigned short *)DefaultAutoRepeatTab;
    else
        art = AutoRepeatTab;
}










