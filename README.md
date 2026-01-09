1. Specyfikacja projektu

I. Oprogramowanie będzie realizować komunikację mikroprocesora z PC poprzez
interfejs asynchroniczny z wykorzystaniem przerwań oraz buforów kołowych.

II. W projekcie zostanie zaimplementowany protokół komunikacyjny pozwalający na
adresowanie ramek, przekazywanie dowolnych danych oraz weryfikację
poprawności danych z uwzględnieniem ich kolejności.

III. Obsługa czujnika TCS34725 odbywać się będzie poprzez interfejs I²C z
wykorzystaniem funkcji nieblokujących wspieranych przez DMA. Dane zapisywane
będą w zadanym interwale czasu zdefiniowanym w milisekundach (uint32) i
umieszczane w buforze cyklicznym o pojemności minimum 600 wpisów, z
możliwością odczytu danych bieżących i archiwalnych.
