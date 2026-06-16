// Юнит-тесты чистых парсеров OBD-II. Не требуют адаптера и UI.
import 'package:flutter_test/flutter_test.dart';
import 'package:obd_scanner/main.dart';

String asciiHex(String s) =>
    s.codeUnits.map((c) => c.toRadixString(16).padLeft(2, '0').toUpperCase()).join();

void main() {
  group('ObdParsers — базовые PID', () {
    test('RPM (010C): 410C1AF8 → 1726', () {
      expect(ObdParsers.rpm('410C1AF8'), 1726);
    });

    test('Скорость (010D): 410D50 → 80', () {
      expect(ObdParsers.speed('410D50'), 80);
    });

    test('Темп. ОЖ (0105): 41055A → 50 (90 - 40)', () {
      expect(ObdParsers.coolant('41055A'), 50);
    });

    test('Нагрузка (0104): 410480 → ~50.2%', () {
      expect(ObdParsers.engineLoad('410480'), closeTo(50.196, 0.01));
    });

    test('Мусорный ответ → null', () {
      expect(ObdParsers.rpm('NODATA'), isNull);
      expect(ObdParsers.speed('SEARCHING'), isNull);
    });

    test('Неверный PID в ответе → null', () {
      // 410D — это скорость, а просим обороты
      expect(ObdParsers.rpm('410D50'), isNull);
    });
  });

  group('ObdParsers — напряжение', () {
    test('ATRV "12.3V" → 12.3', () {
      expect(ObdParsers.voltage('12.3V'), 12.3);
    });
  });

  group('ObdParsers — DTC (Mode 03)', () {
    test('4301330420 → [P0133, P0420]', () {
      expect(ObdParsers.dtc('4301330420'), ['P0133', 'P0420']);
    });

    test('пустые слоты 0000 пропускаются', () {
      expect(ObdParsers.dtc('4301330000'), ['P0133']);
    });

    test('нет ответа режима 43 → пусто', () {
      expect(ObdParsers.dtc('NODATA'), isEmpty);
    });
  });

  group('ObdParsers — VIN (Mode 09 PID 02)', () {
    test('одиночный кадр', () {
      const vin = 'WAUZZZ8K9BA123456';
      final resp = '490201${asciiHex(vin)}';
      expect(ObdParsers.vin(resp), vin);
    });

    test('многокадровый ответ с маркерами строк', () {
      const vin = 'JH4KA8260MC000000';
      final resp = '0:490201${asciiHex(vin.substring(0, 6))}'
          '1:${asciiHex(vin.substring(6, 13))}'
          '2:${asciiHex(vin.substring(13))}';
      expect(ObdParsers.vin(resp), vin);
    });

    test('без эха 4902 → null', () {
      expect(ObdParsers.vin('NODATA'), isNull);
    });
  });

  group('ObdParsers — готовность мониторов (Mode 01 PID 01)', () {
    test('410100072100: лампа выключена, 0 ошибок', () {
      final r = ObdParsers.readiness('410100072100')!;
      expect(r.milOn, isFalse);
      expect(r.dtcCount, 0);
      final cat = r.monitors.firstWhere((m) => m.name == 'Катализатор');
      expect(cat.supported, isTrue);
    });

    test('бит лампы и счётчик DTC', () {
      // A = 0x83 → лампа включена, 3 ошибки
      final r = ObdParsers.readiness('410183000000')!;
      expect(r.milOn, isTrue);
      expect(r.dtcCount, 3);
    });
  });

  group('ObdParsers — стоп-кадр (Mode 02)', () {
    test('420500 5A → данные [0x5A]', () {
      expect(ObdParsers.freezeData('4205005A', '05'), [0x5A]);
    });
  });
}
