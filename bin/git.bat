@echo off
set gp="%Apps_Path%\Git\cmd"

%gp%\git rev-parse --is-inside-work-tree

set Is_Git_Repo=%ERRORLEVEL%

IF %Is_Git_Repo% == 0 (
	%gp%\git config --local http.sslverify "false"
	%gp%\git config --local pull.rebase merges

	%gp%\git config --local alias.co checkout
	%gp%\git config --local alias.br branch
	%gp%\git config --local alias.ci commit
	%gp%\git config --local alias.st status

	%gp%\git config --local alias.ll "log --oneline"
	%gp%\git config --local alias.last "log -1 HEAD --stat"
	%gp%\git config --local alias.cm "commit -m"
	%gp%\git config --local alias.rv "remote -v"
	%gp%\git config --local alias.d "diff"
	%gp%\git config --local alias.gl "config --global -l"
	%gp%\git config --local alias.se "!git rev-list --all | xargs git grep -F"
	%gp%\git config --local alias.dt "difftool -t meld -y"
	%gp%\git config --local alias.mt "mergetool -t meld -y"

	%gp%\git config --local alias.lg "log --graph --abbrev-commit --decorate --all --format=format:'%%C(bold blue)%%h%%C(reset) - %%C(bold green)(%%ar)%%C(reset) %%C(white)%%s%%C(reset) %%C(dim white)- %%an%%C(reset)%%C(auto)%%d%%C(reset)'"
)

start git-bash.exe