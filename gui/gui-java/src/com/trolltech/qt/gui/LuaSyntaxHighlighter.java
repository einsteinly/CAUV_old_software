package com.trolltech.qt.gui;

import java.util.Vector;

import com.trolltech.qt.core.QRegExp;

public class LuaSyntaxHighlighter extends QSyntaxHighlighter {

	private class HighlightingRule {
		public QRegExp pattern;
		public QTextCharFormat format;

		public HighlightingRule(QRegExp pattern, QTextCharFormat format) {
			this.pattern = pattern;
			this.format = format;
		}
	}

	Vector<HighlightingRule> highlightingRules = new Vector<HighlightingRule>();

	QRegExp commentStartExpression;
	QRegExp commentEndExpression;

	QTextCharFormat keywordFormat = new QTextCharFormat();
	QTextCharFormat funcFormat = new QTextCharFormat();
	QTextCharFormat singleLineCommentFormat = new QTextCharFormat();
	QTextCharFormat multiLineCommentFormat = new QTextCharFormat();
	QTextCharFormat quotationFormat = new QTextCharFormat();
	QTextCharFormat squotationFormat = new QTextCharFormat();
	QTextCharFormat numberFormat = new QTextCharFormat();

	public LuaSyntaxHighlighter(QTextDocument document) {
		super(document);

		// keywords
		keywordFormat.setForeground(new QBrush(QColor.blue));
		keywordFormat.setFontWeight(QFont.Weight.Bold.value());
		String[] keywordPatterns = { "\\band\\b", "\\bbreak\\b", "\\bdo\\b",
				"\\belse\\b", "\\belseif\\b", "\\bend\\b", "\\bfalse\\b",
				"\\bfor\\b", "\\bfunction\\b", "\\bif\\b", "\\bin\\b",
				"\\blocal\\b", "\\bnil\\b", "\\bnot\\b", "\\bor\\b",
				"\\brepeat\\b", "\\breturn\\b", "\\bthen\\b", "\\btrue\\b",
				"\\buntil\\b", "\\bwhile\\b" };
		for (String pattern : keywordPatterns) {
			highlightingRules.add(new HighlightingRule(new QRegExp(pattern),
					keywordFormat));
		}

		// functions
		funcFormat.setForeground(new QBrush(QColor.green));
		funcFormat.setFontWeight(QFont.Weight.Bold.value());
		String[] funcPatterns = { "\\bassert\\b", "\\bcollectgarbage\\b",
				"\\bdofile\\b", "\\berror\\b", "\\bnext\\b", "\\bprint\\b",
				"\\bcprint\\b", "\\brawget\\b", "\\brawset\\b",
				"\\btonumber\\b", "\\btostring\\b", "\\btype\\b",
				"\\bloadstring\\b", "\\bpairs\\b", "\\bipairs\\b",
				"\\bpcall\\b", "\\brawequal\\b", "\\brequire\\b",
				"\\bsetfenv\\b", "\\bsetmetatable\\b", "\\bunpack\\b",
				"\\bxpcall\\b", "\\bload\\b", "\\bmodule\\b", "\\bselect\\b",
				"\\bpackage.cpath\\b", "\\bpackage.loaded\\b",
				"\\bpackage.loadlib\\b", "\\bpackage.path\\b",
				"\\bpackage.preload\\b", "\\bpackage.seeall\\b",
				"\\bcoroutine.running\\b", "\\bcoroutine.create\\b",
				"\\bcoroutine.resume\\b", "\\bcoroutine.status\\b",
				"\\bcoroutine.wrap\\b", "\\bcoroutine.yield\\b",
				"\\bstring.byte\\b", "\\bstring.char\\b", "\\bstring.dump\\b",
				"\\bstring.find\\b", "\\bstring.len\\b", "\\bstring.lower\\b",
				"\\bstring.rep\\b", "\\bstring.sub\\b", "\\bstring.upper\\b",
				"\\bstring.format\\b", "\\bstring.gsub\\b",
				"\\bstring.gmatch\\b", "\\bstring.match\\b",
				"\\bstring.reverse\\b", "\\btable.maxn\\b",
				"\\btable.concat\\b", "\\btable.sort\\b", "\\btable.insert\\b",
				"\\btable.remove\\b", "\\bmath.abs\\b", "\\bmath.acos\\b",
				"\\bmath.asin\\b", "\\bmath.atan\\b", "\\bmath.atan2\\b",
				"\\bmath.ceil\\b", "\\bmath.sin\\b", "\\bmath.cos\\b",
				"\\bmath.tan\\b", "\\bmath.deg\\b", "\\bmath.exp\\b",
				"\\bmath.floor\\b", "\\bmath.log\\b", "\\bmath.log10\\b",
				"\\bmath.max\\b", "\\bmath.min\\b", "\\bmath.fmod\\b",
				"\\bmath.modf\\b", "\\bmath.cosh\\b", "\\bmath.sinh\\b",
				"\\bmath.tanh\\b", "\\bmath.pow\\b", "\\bmath.rad\\b",
				"\\bmath.sqrt\\b", "\\bmath.frexp\\b", "\\bmath.ldexp\\b",
				"\\bmath.random\\b", "\\bmath.randomseed\\b", "\\bmath.pi\\b",
				"\\bio.stdin\\b", "\\bio.stdout\\b", "\\bio.stderr\\b",
				"\\bio.close\\b", "\\bio.flush\\b", "\\bio.input\\b",
				"\\bio.lines\\b", "\\bio.open\\b", "\\bio.output\\b",
				"\\bio.popen\\b", "\\bio.read\\b", "\\bio.tmpfile\\b",
				"\\bio.type\\b", "\\bio.write\\b", "\\bos.clock\\b",
				"\\bos.date\\b", "\\bos.difftime\\b", "\\bos.execute\\b",
				"\\bos.exit\\b", "\\bos.getenv\\b", "\\bos.remove\\b",
				"\\bos.rename\\b", "\\bos.setlocale\\b", "\\bos.time\\b",
				"\\bos.tmpname\\b", "\\bdebug.debug\\b", "\\bdebug.gethook\\b",
				"\\bdebug.getinfo\\b", "\\bdebug.getlocal\\b",
				"\\bdebug.getupvalue\\b", "\\bdebug.setlocal\\b",
				"\\bdebug.setupvalue\\b", "\\bdebug.sethook\\b",
				"\\bdebug.traceback\\b", "\\bdebug.getfenv\\b",
				"\\bdebug.getmetatable\\b", "\\bdebug.getregistry\\b",
				"\\bdebug.setfenv\\b", "\\bdebug.setmetatable\\b" };
		for (String pattern : funcPatterns) {
			highlightingRules.add(new HighlightingRule(new QRegExp(pattern),
					funcFormat));
		}

		// single line comments
		singleLineCommentFormat.setForeground(new QBrush(QColor.darkGray));
		highlightingRules.add(new HighlightingRule(new QRegExp(
				"--[^\\[\\]][^\n]*"), singleLineCommentFormat));

		// quotations
		quotationFormat.setForeground(new QBrush(QColor.red));
		highlightingRules.add(new HighlightingRule(new QRegExp("\"[^\"]*\""),
				quotationFormat));
		highlightingRules.add(new HighlightingRule(new QRegExp("'[^']*'"),
				quotationFormat));

		// numbers
		numberFormat.setForeground(new QBrush(QColor.red));
		numberFormat.setFontWeight(QFont.Weight.Bold.value());
		highlightingRules.add(new HighlightingRule(new QRegExp(
				"[-]?\\d{1,}[\\.]?\\d*[eE]{1}[-]?\\d{1,}"), numberFormat));
		highlightingRules.add(new HighlightingRule(new QRegExp(
				"[-]?\\d{1,}[\\.]?\\d*"), numberFormat));

		// comments
		commentStartExpression = new QRegExp("--\\[([=]*)\\[");
		multiLineCommentFormat.setForeground(new QBrush(QColor.darkGray));
	}

	protected void highlightBlock(String text) {

		   for (HighlightingRule rule : highlightingRules) {

		      QRegExp expression = new QRegExp(rule.pattern);
		      int index = expression.indexIn(text);

		      while (index >= 0) {

		         int length = expression.matchedLength();
		         setFormat(index, length, rule.format);
		         index = expression.indexIn (text, index + length);
		      }
		   }

		   setCurrentBlockState (0);

		   int startIndex = 0;

		   if (previousBlockState () != 1) {

		       startIndex = commentStartExpression.indexIn(text);
		       String comment  = "--\\]";
		       comment += commentStartExpression.cap (1) + "\\]";
		       commentEndExpression = new QRegExp (comment);
		   }

		   while (startIndex >= 0) {

		      int endIndex = commentEndExpression.indexIn(text, startIndex);
		      int commentLength;

		      if (endIndex == -1) {

		         setCurrentBlockState (1);
		         commentLength = text.length () - startIndex;
		      }
		      else {

		         commentLength = endIndex - startIndex
		            + commentEndExpression.matchedLength ();
		      }

		      setFormat (startIndex, commentLength, multiLineCommentFormat);
		      startIndex = commentStartExpression.indexIn (text, startIndex + commentLength);
		   }
	}
};
